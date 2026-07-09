from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from storage.lfs import record_payload_status
from storage.records_index import load_records_index, records_index_by_id
from storage.revisions import (
    active_cost_path,
    active_provenance_path,
    active_revision_id,
    active_traces_dir,
    descendants_for_record,
    list_record_revisions,
)
from storage.runs import RunStore
from storage.subcat_index import load_subcat_index, subcat_index_signature
from viewer.api.agent_harness import agent_harness_from_record
from viewer.api.schemas import (
    DatasetEntryResponse,
    RecordDetailResponse,
    RecordHistoryResponse,
    RecordHistoryRevisionResponse,
    RecordSummaryResponse,
    WorkbenchEntryResponse,
)
from viewer.api.store_components import ViewerStoreComponent
from viewer.api.store_values import (
    _coerce_int,
    _coerce_rating,
    _coerce_string,
    _cost_totals,
    _cost_turn_count,
    _effective_rating,
    _normalize_sdk_package_value,
    _parse_sort_key,
    _thinking_level_from_provenance,
)


class ViewerRecordsStore(ViewerStoreComponent):
    def _records_index_row(self, record_id: str) -> dict[str, Any] | None:
        """Look up a record's index row, caching the parsed 16MB index by mtime.

        ``find_record_index_row`` re-reads and re-parses the entire
        ``records_index.jsonl`` on every call. For endpoints that resolve many
        unhydrated records (``list_staging_entries``, workbench shard rebuilds)
        that is an O(N x whole-file) blowup that saturates the thread pool. This
        memoizes the parsed id->row map and invalidates on the index file mtime.
        """
        index_path = self.repo.layout.records_index_path
        try:
            mtime_ns: int | None = index_path.stat().st_mtime_ns
        except OSError:
            mtime_ns = None

        with self._records_index_cache_guard:
            cached = self._records_index_cache
            if cached is not None and cached[0] == mtime_ns:
                row = cached[1].get(record_id)
                return row if isinstance(row, dict) else None

        lookup = records_index_by_id(self.repo)
        with self._records_index_cache_guard:
            self._records_index_cache = (mtime_ns, lookup)
        row = lookup.get(record_id)
        return row if isinstance(row, dict) else None

    def _read_jsonl(self, path: Path) -> list[dict[str, Any]]:
        if not path.exists():
            return []
        rows: list[dict[str, Any]] = []
        for line in path.read_text(encoding="utf-8").splitlines():
            if not line.strip():
                continue
            try:
                parsed = json.loads(line)
            except json.JSONDecodeError:
                continue
            if isinstance(parsed, dict):
                rows.append(parsed)
        return rows

    def _read_run_results(self, run_id: str) -> list[dict[str, Any]]:
        return RunStore(self.repo).read_latest_results(run_id, key="row_id")

    def _run_result_for_record(self, run_id: str, record_id: str) -> dict[str, Any] | None:
        results_path = self.repo.layout.run_results_path(run_id)
        try:
            mtime_ns = results_path.stat().st_mtime_ns
        except OSError:
            mtime_ns = None

        cached_lookup: dict[str, dict[str, Any]] | None = None
        with self._run_results_cache_guard:
            cached = self._run_results_cache.get(run_id)
            if cached is not None and cached[0] == mtime_ns:
                cached_lookup = cached[1]

        if cached_lookup is None:
            lookup: dict[str, dict[str, Any]] = {}
            for row in self._read_run_results(run_id):
                row_record_id = _coerce_string(row.get("record_id"))
                if row_record_id is None:
                    continue
                lookup[row_record_id] = row
            with self._run_results_cache_guard:
                self._run_results_cache[run_id] = (mtime_ns, lookup)
            cached_lookup = lookup

        row = cached_lookup.get(record_id)
        return row if isinstance(row, dict) else None

    def _read_text(self, path: Path) -> str | None:
        if not path.exists() or not path.is_file():
            return None
        try:
            return path.read_text(encoding="utf-8")
        except OSError:
            return None

    def _history_revision_response(
        self,
        *,
        record_id: str,
        revision_id: str,
        active: bool,
        revision: dict[str, Any],
        prompt: str | None,
        trace_available: bool = True,
    ) -> RecordHistoryRevisionResponse:
        generation = (
            revision.get("generation") if isinstance(revision.get("generation"), dict) else {}
        )
        source = revision.get("source") if isinstance(revision.get("source"), dict) else {}
        parent = revision.get("parent") if isinstance(revision.get("parent"), dict) else {}
        run_summary = (
            revision.get("run_summary") if isinstance(revision.get("run_summary"), dict) else {}
        )
        revision_dir = self.repo.layout.record_revision_dir(record_id, revision_id)
        cost_path = revision_dir / "cost.json"
        cost = self.repo.read_json(cost_path) if cost_path.exists() else None
        total_cost_usd, _, _ = _cost_totals(cost)
        return RecordHistoryRevisionResponse(
            record_id=record_id,
            revision_id=revision_id,
            active=active,
            created_at=_coerce_string(revision.get("created_at")),
            prompt_preview=(prompt or "").replace("\n", " ")[:160],
            provider=_coerce_string(generation.get("provider")),
            model_id=_coerce_string(generation.get("model_id")),
            run_id=_coerce_string(source.get("run_id")),
            parent_record_id=_coerce_string(parent.get("record_id")),
            parent_revision_id=_coerce_string(parent.get("revision_id")),
            status=_coerce_string(run_summary.get("final_status")),
            total_cost_usd=total_cost_usd,
            has_cost=cost_path.exists(),
            has_traces=trace_available
            and (revision_dir / "traces").is_dir()
            and any((revision_dir / "traces").iterdir()),
            has_model=(revision_dir / "model.py").exists(),
            has_provenance=(revision_dir / "provenance.json").exists(),
        )

    def _record_creator_fields(self, record: dict[str, Any]) -> tuple[str | None, str | None]:
        creator = record.get("creator")
        if not isinstance(creator, dict):
            return None, None
        mode = _coerce_string(creator.get("mode"))
        agent = _coerce_string(creator.get("agent")) if mode == "external_agent" else None
        return mode, agent

    def _record_trace_available(self, record: dict[str, Any]) -> bool:
        creator = record.get("creator")
        return not (isinstance(creator, dict) and creator.get("trace_available") is False)

    def _record_has_traces(self, record_id: str, record: dict[str, Any]) -> bool:
        if not self._record_trace_available(record):
            return False
        traces_dir = active_traces_dir(self.repo, record_id, record=record)
        return traces_dir.is_dir() and any(traces_dir.iterdir())

    def _record_summary_from_index_row(
        self,
        record_id: str,
        row: dict[str, Any],
        *,
        payload_status: str | None = None,
    ) -> RecordSummaryResponse:
        primary_rating = _coerce_rating(row.get("rating"))
        secondary_rating = _coerce_rating(row.get("secondary_rating"))
        collections = row.get("collections")
        return RecordSummaryResponse(
            record_id=record_id,
            title=str(row.get("title") or record_id),
            prompt_preview=str(row.get("prompt_preview") or ""),
            rating=primary_rating,
            secondary_rating=secondary_rating,
            effective_rating=_effective_rating(primary_rating, secondary_rating),
            author=_coerce_string(row.get("author")),
            rated_by=_coerce_string(row.get("rated_by")),
            secondary_rated_by=_coerce_string(row.get("secondary_rated_by")),
            created_at=_coerce_string(row.get("created_at")),
            updated_at=_coerce_string(row.get("updated_at")),
            viewer_asset_updated_at=None,
            sdk_package=_normalize_sdk_package_value(row.get("sdk_package")),
            provider=_coerce_string(row.get("provider")),
            model_id=_coerce_string(row.get("model_id")),
            creator_mode=_coerce_string(row.get("creator_mode")),
            external_agent=_coerce_string(row.get("external_agent")),
            agent_harness=_coerce_string(row.get("agent_harness")) or "articraft",
            has_traces=bool(row.get("has_traces", False)),
            thinking_level=_coerce_string(row.get("thinking_level")),
            turn_count=_coerce_int(row.get("turn_count")),
            input_tokens=_coerce_int(row.get("input_tokens")),
            output_tokens=_coerce_int(row.get("output_tokens")),
            total_cost_usd=(
                float(row.get("total_cost_usd"))
                if isinstance(row.get("total_cost_usd"), (int, float))
                else None
            ),
            category_slug=_coerce_string(row.get("category_slug")),
            run_id=_coerce_string(row.get("run_id")),
            run_status=_coerce_string(row.get("run_status")),
            run_message=None,
            active_revision_id=_coerce_string(row.get("active_revision_id")),
            origin_record_id=_coerce_string(row.get("origin_record_id")),
            parent_record_id=_coerce_string(row.get("parent_record_id")),
            revision_count=_coerce_int(row.get("revision_count")) or 0,
            has_history=bool(row.get("has_history", False)),
            collections=(
                [str(item) for item in collections] if isinstance(collections, list) else []
            ),
            materialization_status=None,
            has_compile_report=bool(row.get("has_compile_report", False)),
            has_provenance=bool(row.get("has_provenance", False)),
            has_cost=bool(row.get("has_cost", False)),
            payload_status=(
                payload_status
                if payload_status is not None
                else record_payload_status(self.repo, record_id)
            ),
        )

    def _record_summary(
        self,
        record_id: str,
        summary_cache: dict[str, RecordSummaryResponse | None] | None = None,
    ) -> RecordSummaryResponse | None:
        if summary_cache is not None and record_id in summary_cache:
            return summary_cache[record_id]

        record_path = self.repo.layout.record_metadata_path(record_id)
        record = self.repo.read_json(record_path)
        if record is None:
            index_row = self._records_index_row(record_id)
            if index_row is not None:
                summary = self._record_summary_from_index_row(record_id, index_row)
                if summary_cache is not None:
                    summary_cache[record_id] = summary
                return summary
            if summary_cache is not None:
                summary_cache[record_id] = None
            return None

        display = record.get("display") or {}
        source = record.get("source") or {}
        artifacts = record.get("artifacts") or {}
        record_dir = self.repo.layout.record_dir(record_id)

        compile_path = self.repo.layout.record_materialization_compile_report_path(record_id)
        provenance_path = active_provenance_path(self.repo, record_id, record=record)
        cost_name = artifacts.get("cost_json")
        provenance = self.repo.read_json(provenance_path)
        cost_path = (
            record_dir / str(cost_name)
            if cost_name
            else active_cost_path(self.repo, record_id, record=record)
        )
        cost = self.repo.read_json(cost_path) if cost_path.exists() else None
        materialization_status = self.materialization._materialization_status_for_record(record_id)
        creator_mode, external_agent = self._record_creator_fields(record)
        agent_harness = agent_harness_from_record(record)
        has_traces = self._record_has_traces(record_id, record)
        lineage = record.get("lineage") if isinstance(record.get("lineage"), dict) else {}
        active_id = _coerce_string(record.get("active_revision_id"))
        revision_root = self.repo.layout.record_revisions_dir(record_id)
        revision_count = (
            len([path for path in revision_root.iterdir() if path.is_dir()])
            if revision_root.is_dir()
            else 0
        )

        turn_count: int | None = None
        thinking_level: str | None = None
        run_status: str | None = None
        run_message: str | None = None
        if isinstance(provenance, dict):
            run_summary = provenance.get("run_summary")
            if isinstance(run_summary, dict):
                turn_count = _coerce_int(run_summary.get("turn_count"))
                run_status = _coerce_string(run_summary.get("final_status"))
            thinking_level = _thinking_level_from_provenance(provenance)

        total_cost_usd, input_tokens, output_tokens = _cost_totals(cost)
        if turn_count is None:
            turn_count = _cost_turn_count(cost)

        run_id = _coerce_string(source.get("run_id")) if isinstance(source, dict) else None
        if run_id is not None:
            run_result = self._run_result_for_record(run_id, record_id)
            if isinstance(run_result, dict):
                run_status = _coerce_string(run_result.get("status")) or run_status
                run_message = _coerce_string(run_result.get("message"))

        primary_rating = _coerce_rating(record.get("rating"))
        secondary_rating = _coerce_rating(record.get("secondary_rating"))
        picture_category, picture_subcategory = self.picture.resolve_subcategory(
            record_id,
            origin_record_id=_coerce_string(lineage.get("origin_record_id")),
            parent_record_id=_coerce_string(lineage.get("parent_record_id")),
            category_slug=_coerce_string(record.get("category_slug")),
        )
        summary = RecordSummaryResponse(
            record_id=record_id,
            title=str(display.get("title") or record_id),
            prompt_preview=str(display.get("prompt_preview") or ""),
            rating=primary_rating,
            secondary_rating=secondary_rating,
            effective_rating=_effective_rating(primary_rating, secondary_rating),
            author=_coerce_string(record.get("author")),
            rated_by=_coerce_string(record.get("rated_by")),
            secondary_rated_by=_coerce_string(record.get("secondary_rated_by")),
            created_at=record.get("created_at"),
            updated_at=record.get("updated_at"),
            viewer_asset_updated_at=self.materialization._viewer_asset_updated_at_for_record(
                record_id
            ),
            sdk_package=_normalize_sdk_package_value(record.get("sdk_package")),
            provider=record.get("provider"),
            model_id=record.get("model_id"),
            creator_mode=creator_mode,
            external_agent=external_agent,
            agent_harness=agent_harness,
            has_traces=has_traces,
            thinking_level=thinking_level,
            turn_count=turn_count,
            input_tokens=input_tokens,
            output_tokens=output_tokens,
            total_cost_usd=total_cost_usd,
            category_slug=record.get("category_slug"),
            run_id=run_id,
            run_status=run_status,
            run_message=run_message,
            active_revision_id=active_id,
            origin_record_id=_coerce_string(lineage.get("origin_record_id")),
            parent_record_id=_coerce_string(lineage.get("parent_record_id")),
            picture_category=picture_category,
            picture_subcategory=picture_subcategory,
            revision_count=revision_count,
            has_history=revision_count > 1
            or _coerce_string(lineage.get("parent_record_id")) is not None,
            collections=[str(item) for item in record.get("collections", [])],
            materialization_status=materialization_status,
            has_compile_report=compile_path.exists(),
            has_provenance=provenance_path.exists(),
            has_cost=cost_path.exists(),
            payload_status=record_payload_status(self.repo, record_id),
        )
        if summary_cache is not None:
            summary_cache[record_id] = summary
        return summary

    def _record_browser_summary(
        self,
        record_id: str,
        summary_cache: dict[str, RecordSummaryResponse | None] | None = None,
    ) -> RecordSummaryResponse | None:
        if summary_cache is not None and record_id in summary_cache:
            return summary_cache[record_id]

        record_path = self.repo.layout.record_metadata_path(record_id)
        record = self.repo.read_json(record_path)
        if record is None:
            index_row = self._records_index_row(record_id)
            if index_row is not None:
                summary = self._record_summary_from_index_row(record_id, index_row)
                if summary_cache is not None:
                    summary_cache[record_id] = summary
                return summary
            if summary_cache is not None:
                summary_cache[record_id] = None
            return None

        display = record.get("display") or {}
        source = record.get("source") or {}
        artifacts = record.get("artifacts") or {}
        record_dir = self.repo.layout.record_dir(record_id)

        compile_path = self.repo.layout.record_materialization_compile_report_path(record_id)
        provenance_path = active_provenance_path(self.repo, record_id, record=record)
        cost_name = artifacts.get("cost_json")
        cost_path = (
            record_dir / str(cost_name)
            if cost_name
            else active_cost_path(self.repo, record_id, record=record)
        )
        provenance = self.repo.read_json(provenance_path) if provenance_path.exists() else None
        cost = self.repo.read_json(cost_path) if cost_path and cost_path.exists() else None
        creator_mode, external_agent = self._record_creator_fields(record)
        agent_harness = agent_harness_from_record(record)
        has_traces = self._record_has_traces(record_id, record)
        lineage = record.get("lineage") if isinstance(record.get("lineage"), dict) else {}
        active_id = _coerce_string(record.get("active_revision_id"))
        revision_root = self.repo.layout.record_revisions_dir(record_id)
        revision_count = (
            len([path for path in revision_root.iterdir() if path.is_dir()])
            if revision_root.is_dir()
            else 0
        )

        turn_count: int | None = None
        thinking_level: str | None = None
        if isinstance(provenance, dict):
            run_summary = provenance.get("run_summary")
            if isinstance(run_summary, dict):
                turn_count = _coerce_int(run_summary.get("turn_count"))
            thinking_level = _thinking_level_from_provenance(provenance)

        total_cost_usd, input_tokens, output_tokens = _cost_totals(cost)
        if turn_count is None:
            turn_count = _cost_turn_count(cost)

        run_id = _coerce_string(source.get("run_id")) if isinstance(source, dict) else None
        primary_rating = _coerce_rating(record.get("rating"))
        secondary_rating = _coerce_rating(record.get("secondary_rating"))
        picture_category, picture_subcategory = self.picture.resolve_subcategory(
            record_id,
            origin_record_id=_coerce_string(lineage.get("origin_record_id")),
            parent_record_id=_coerce_string(lineage.get("parent_record_id")),
            category_slug=_coerce_string(record.get("category_slug")),
        )
        summary = RecordSummaryResponse(
            record_id=record_id,
            title=str(display.get("title") or record_id),
            prompt_preview=str(display.get("prompt_preview") or ""),
            rating=primary_rating,
            secondary_rating=secondary_rating,
            effective_rating=_effective_rating(primary_rating, secondary_rating),
            author=_coerce_string(record.get("author")),
            rated_by=_coerce_string(record.get("rated_by")),
            secondary_rated_by=_coerce_string(record.get("secondary_rated_by")),
            created_at=record.get("created_at"),
            updated_at=record.get("updated_at"),
            viewer_asset_updated_at=None,
            sdk_package=_normalize_sdk_package_value(record.get("sdk_package")),
            provider=record.get("provider"),
            model_id=record.get("model_id"),
            creator_mode=creator_mode,
            external_agent=external_agent,
            agent_harness=agent_harness,
            has_traces=has_traces,
            thinking_level=thinking_level,
            turn_count=turn_count,
            input_tokens=input_tokens,
            output_tokens=output_tokens,
            total_cost_usd=total_cost_usd,
            category_slug=record.get("category_slug"),
            run_id=run_id,
            run_status=None,
            run_message=None,
            active_revision_id=active_id,
            origin_record_id=_coerce_string(lineage.get("origin_record_id")),
            parent_record_id=_coerce_string(lineage.get("parent_record_id")),
            picture_category=picture_category,
            picture_subcategory=picture_subcategory,
            revision_count=revision_count,
            has_history=revision_count > 1
            or _coerce_string(lineage.get("parent_record_id")) is not None,
            collections=[str(item) for item in record.get("collections", [])],
            materialization_status=None,
            has_compile_report=compile_path.exists(),
            has_provenance=provenance_path.exists(),
            has_cost=cost_path.exists(),
            payload_status=record_payload_status(self.repo, record_id),
        )
        if summary_cache is not None:
            summary_cache[record_id] = summary
        return summary

    def _record_detail(
        self,
        record_id: str,
        summary_cache: dict[str, RecordSummaryResponse | None] | None = None,
    ) -> RecordDetailResponse | None:
        summary = self._record_summary(record_id, summary_cache=summary_cache)
        if summary is None:
            return None

        record = self.repo.read_json(self.repo.layout.record_metadata_path(record_id))
        if record is None:
            if summary.payload_status != "hydrated":
                return RecordDetailResponse(
                    summary=summary,
                    record=None,
                    compile_report=None,
                    provenance=None,
                    cost=None,
                )
            return None
        artifacts = record.get("artifacts") or {}
        record_dir = self.repo.layout.record_dir(record_id)
        compile_path = self.repo.layout.record_materialization_compile_report_path(record_id)
        cost_name = artifacts.get("cost_json")
        compile_report = self.repo.read_json(compile_path)
        provenance = self.repo.read_json(
            active_provenance_path(self.repo, record_id, record=record)
        )
        cost_path = (
            record_dir / str(cost_name)
            if cost_name
            else active_cost_path(self.repo, record_id, record=record)
        )
        cost = self.repo.read_json(cost_path) if cost_path.exists() else None

        return RecordDetailResponse(
            summary=summary,
            record=record,
            compile_report=compile_report,
            provenance=provenance,
            cost=cost,
        )

    def record_history(self, record_id: str) -> RecordHistoryResponse | None:
        record = self.repo.read_json(self.repo.layout.record_metadata_path(record_id))
        if not isinstance(record, dict):
            index_row = self._records_index_row(record_id)
            if index_row is None:
                return None
            return RecordHistoryResponse(
                record_id=record_id,
                active_revision_id=_coerce_string(index_row.get("active_revision_id")),
                ancestors=[],
                revisions=[],
                descendants=[],
            )
        active_id = active_revision_id(self.repo, record_id, record=record)
        revisions = [
            self._history_revision_response(
                record_id=row.record_id,
                revision_id=row.revision_id,
                active=row.active,
                revision=row.revision,
                prompt=row.prompt,
                trace_available=self._record_trace_available(record),
            )
            for row in list_record_revisions(self.repo, record_id)
        ]

        ancestors: list[RecordHistoryRevisionResponse] = []
        lineage = record.get("lineage") if isinstance(record.get("lineage"), dict) else {}
        parent_record_id = _coerce_string(lineage.get("parent_record_id"))
        parent_revision_id = _coerce_string(lineage.get("parent_revision_id"))
        seen: set[str] = set()
        while parent_record_id and parent_revision_id and parent_record_id not in seen:
            seen.add(parent_record_id)
            parent_revision_dir = self.repo.layout.record_revision_dir(
                parent_record_id,
                parent_revision_id,
            )
            parent_revision = (
                self.repo.read_json(parent_revision_dir / "revision.json", default={}) or {}
            )
            parent_record = self.repo.read_json(
                self.repo.layout.record_metadata_path(parent_record_id),
                default={},
            )
            prompt = self._read_text(parent_revision_dir / "prompt.txt")
            ancestors.append(
                self._history_revision_response(
                    record_id=parent_record_id,
                    revision_id=parent_revision_id,
                    active=False,
                    revision=parent_revision if isinstance(parent_revision, dict) else {},
                    prompt=prompt,
                    trace_available=(
                        self._record_trace_available(parent_record)
                        if isinstance(parent_record, dict)
                        else True
                    ),
                )
            )
            parent_lineage = (
                parent_record.get("lineage")
                if isinstance(parent_record, dict)
                and isinstance(parent_record.get("lineage"), dict)
                else {}
            )
            parent_record_id = _coerce_string(parent_lineage.get("parent_record_id"))
            parent_revision_id = _coerce_string(parent_lineage.get("parent_revision_id"))

        summary_cache: dict[str, RecordSummaryResponse | None] = {}
        descendants = [
            summary
            for descendant_id, _record in descendants_for_record(self.repo, record_id)
            if (summary := self._record_summary(descendant_id, summary_cache=summary_cache))
            is not None
        ]
        return RecordHistoryResponse(
            record_id=record_id,
            active_revision_id=active_id,
            ancestors=ancestors,
            revisions=revisions,
            descendants=descendants,
        )

    def _workbench_snapshot_signature(self) -> int | None:
        # Prefer the workbench-scoped per-小类 shard index when present: it is the
        # fast, parallel-safe membership source for the picture/template pipeline.
        shard_sig = subcat_index_signature(self.repo)
        if shard_sig is not None:
            return shard_sig
        try:
            return self.repo.layout.records_index_path.stat().st_mtime_ns
        except OSError:
            return None

    def _build_workbench_summary(
        self, record_id: str, row: dict[str, Any]
    ) -> RecordSummaryResponse:
        memo = self._workbench_payload_status_memo
        payload_status = memo.get(record_id)
        if payload_status is None:
            payload_status = record_payload_status(self.repo, record_id)
            memo[record_id] = payload_status
        summary = self._record_summary_from_index_row(record_id, row, payload_status=payload_status)
        picture_category, picture_subcategory = self.picture.resolve_subcategory(
            record_id,
            origin_record_id=_coerce_string(row.get("origin_record_id")),
            parent_record_id=_coerce_string(row.get("parent_record_id")),
            category_slug=_coerce_string(row.get("category_slug")),
        )
        if picture_category is not None or picture_subcategory is not None:
            summary = summary.model_copy(
                update={
                    "picture_category": picture_category,
                    "picture_subcategory": picture_subcategory,
                }
            )
        return summary

    def _rebuild_workbench_entries_from_shards(self) -> list[WorkbenchEntryResponse]:
        # Workbench-scoped per-小类 shards (data/index/subcat/*.jsonl) are the membership
        # source for the ~1k picture/template-pipeline records. Each row already carries
        # its 小类 (resolved from the per-record binding at reconcile time), so the grid
        # shows the right 小类 without touching the 16MB global records_index.
        entries: list[WorkbenchEntryResponse] = []
        for row in load_subcat_index(self.repo):
            record_id = str(row.get("record_id") or "")
            if not record_id:
                continue
            summary = self._record_summary(record_id)
            if summary is None:
                continue
            picture_category = _coerce_string(row.get("picture_category"))
            picture_subcategory = _coerce_string(row.get("picture_subcategory"))
            if picture_category is not None or picture_subcategory is not None:
                summary = summary.model_copy(
                    update={
                        "picture_category": picture_category,
                        "picture_subcategory": picture_subcategory,
                    }
                )
            entries.append(
                WorkbenchEntryResponse(
                    record_id=record_id,
                    added_at=str(row.get("added_at") or ""),
                    label=_coerce_string(row.get("label")),
                    tags=[],
                    archived=bool(row.get("archived", False)),
                    record=summary,
                )
            )
        return sorted(entries, key=lambda entry: _parse_sort_key(entry.added_at), reverse=True)

    def _rebuild_workbench_entries(self) -> list[WorkbenchEntryResponse]:
        if subcat_index_signature(self.repo) is not None:
            return self._rebuild_workbench_entries_from_shards()
        # Fast path: when a records index is present, membership and the record summary
        # both come from records_index.jsonl. Its `collections` field mirrors the
        # per-record workbench sidecars (the canonical index is rebuilt on every rating
        # change, delete, and external finalize), so we avoid
        # CollectionStore.load_workbench()'s full records-root scan, which reads a sidecar
        # for every record on disk. Only the workbench-sized set of sidecars is read here,
        # for label/tags/added_at/archived. materialization_status and
        # viewer_asset_updated_at are intentionally left at their defaults — the grid does
        # not show them and the per-record /summary fetch supplies accurate values on
        # selection.
        index_rows = {
            str(row.get("record_id")): row
            for row in load_records_index(self.repo)
            if row.get("record_id")
        }
        if not index_rows:
            return self._rebuild_workbench_entries_from_sidecars()

        entries: list[WorkbenchEntryResponse] = []
        for record_id, row in index_rows.items():
            collections = row.get("collections")
            if not (isinstance(collections, list) and "workbench" in collections):
                continue
            sidecar = self.repo.read_json(
                self.repo.layout.record_workbench_entry_path(record_id),
                default={},
            )
            if not isinstance(sidecar, dict):
                sidecar = {}
            entries.append(
                WorkbenchEntryResponse(
                    record_id=record_id,
                    added_at=str(sidecar.get("added_at", "")),
                    label=sidecar.get("label"),
                    tags=[str(tag) for tag in sidecar.get("tags", [])],
                    archived=bool(sidecar.get("archived", False)),
                    record=self._build_workbench_summary(record_id, row),
                )
            )
        return sorted(entries, key=lambda entry: _parse_sort_key(entry.added_at), reverse=True)

    def _rebuild_workbench_entries_from_sidecars(self) -> list[WorkbenchEntryResponse]:
        # Fallback for repos without a records index (fresh repos, tests): membership and
        # metadata come from the authoritative per-record workbench sidecars, and the full
        # record summary is read from disk — matching the pre-index behavior exactly.
        workbench = self.collection_store.load_workbench() or {"entries": []}
        entries: list[WorkbenchEntryResponse] = []
        for item in workbench.get("entries", []):
            record_id = str(item.get("record_id", ""))
            if not record_id:
                continue
            entries.append(
                WorkbenchEntryResponse(
                    record_id=record_id,
                    added_at=str(item.get("added_at", "")),
                    label=item.get("label"),
                    tags=[str(tag) for tag in item.get("tags", [])],
                    archived=bool(item.get("archived", False)),
                    record=self._record_summary(record_id),
                )
            )
        return sorted(entries, key=lambda entry: _parse_sort_key(entry.added_at), reverse=True)

    def list_workbench_entries(
        self,
        summary_cache: dict[str, RecordSummaryResponse | None] | None = None,
    ) -> list[WorkbenchEntryResponse]:
        signature = self._workbench_snapshot_signature()
        if signature is None:
            # No records index to invalidate against — always rebuild from the
            # authoritative sidecars rather than risk serving a stale snapshot.
            entries = self._rebuild_workbench_entries()
        else:
            with self._workbench_snapshot_guard:
                cached = self._workbench_summary_snapshot
                if cached is None or cached[0] != signature:
                    cached = (signature, self._rebuild_workbench_entries())
                    self._workbench_summary_snapshot = cached
                entries = cached[1]
        if summary_cache is not None:
            for entry in entries:
                if entry.record is not None:
                    summary_cache[entry.record_id] = entry.record
        return entries

    def list_dataset_entries(
        self,
        summary_cache: dict[str, RecordSummaryResponse | None] | None = None,
        *,
        include_records: bool = True,
    ) -> list[DatasetEntryResponse]:
        entries: list[DatasetEntryResponse] = []
        for item in self.dataset_store.list_entries():
            record_id = str(item.get("record_id", ""))
            dataset_id = str(item.get("dataset_id", ""))
            category_slug = str(item.get("category_slug", ""))
            promoted_at = str(item.get("promoted_at", ""))
            if not record_id or not dataset_id:
                continue
            entries.append(
                DatasetEntryResponse(
                    record_id=record_id,
                    dataset_id=dataset_id,
                    category_slug=category_slug,
                    promoted_at=promoted_at,
                    record=(
                        self._record_summary(record_id, summary_cache=summary_cache)
                        if include_records
                        else None
                    ),
                )
            )
        return entries
