import { useCallback, useEffect, useMemo, type JSX } from "react";

import { useQuery } from "@tanstack/react-query";

import { pictureFileUrl } from "@/lib/api";
import type { PictureSubcategory, RecordSummary } from "@/lib/types";
import { useWorkbenchRecords } from "@/lib/use-workbench-records";
import { useViewer, useViewerDispatch } from "@/lib/viewer-context";
import { pictureSubcategoriesQueryOptions } from "@/lib/viewer-queries";
import { RecordListItem } from "@/components/browser/RecordListItem";

const UNGROUPED_KEY = "__ungrouped__";

type RecordGroup = {
  key: string;
  title: string;
  referenceImages: string[];
  originals: RecordSummary[];
  variants: RecordSummary[];
};

interface GroupedRecordListProps {
  onVisibleIdsChange?: (ids: string[]) => void;
  onCountsChange?: (counts: { visible: number; total: number }) => void;
}

function isVariant(record: RecordSummary): boolean {
  return record.parent_record_id != null && record.parent_record_id !== record.record_id;
}

function groupKeyForRecord(record: RecordSummary): string {
  if (record.picture_category && record.picture_subcategory) {
    return `${record.picture_category}/${record.picture_subcategory}`;
  }
  return UNGROUPED_KEY;
}

export function GroupedRecordList({
  onVisibleIdsChange,
  onCountsChange,
}: GroupedRecordListProps): JSX.Element {
  const { bootstrap, selectedRecordId, multiSelection } = useViewer();
  const dispatch = useViewerDispatch();
  const { records, sourceRecords } = useWorkbenchRecords();
  const { data: pictureSubcategories } = useQuery(pictureSubcategoriesQueryOptions());

  const referenceImagesByKey = useMemo(() => {
    const map = new Map<string, PictureSubcategory>();
    for (const entry of pictureSubcategories ?? []) {
      map.set(entry.key, entry);
    }
    return map;
  }, [pictureSubcategories]);

  const groups = useMemo<RecordGroup[]>(() => {
    const byKey = new Map<string, RecordSummary[]>();
    for (const record of records) {
      const key = groupKeyForRecord(record);
      const bucket = byKey.get(key);
      if (bucket) {
        bucket.push(record);
      } else {
        byKey.set(key, [record]);
      }
    }

    const result: RecordGroup[] = [];
    for (const [key, groupRecords] of byKey.entries()) {
      if (key === UNGROUPED_KEY) {
        continue;
      }
      const meta = referenceImagesByKey.get(key);
      result.push({
        key,
        title: key.replace("/", " / "),
        referenceImages: meta?.reference_images ?? [],
        originals: groupRecords.filter((record) => !isVariant(record)),
        variants: groupRecords.filter((record) => isVariant(record)),
      });
    }
    result.sort((left, right) => left.key.localeCompare(right.key));

    const ungrouped = byKey.get(UNGROUPED_KEY);
    if (ungrouped && ungrouped.length > 0) {
      result.push({
        key: UNGROUPED_KEY,
        title: "Ungrouped",
        referenceImages: [],
        originals: ungrouped.filter((record) => !isVariant(record)),
        variants: ungrouped.filter((record) => isVariant(record)),
      });
    }
    return result;
  }, [records, referenceImagesByKey]);

  const visibleIds = useMemo(() => {
    const ids: string[] = [];
    for (const group of groups) {
      for (const record of group.originals) ids.push(record.record_id);
      for (const record of group.variants) ids.push(record.record_id);
    }
    return ids;
  }, [groups]);

  useEffect(() => {
    onVisibleIdsChange?.(visibleIds);
  }, [onVisibleIdsChange, visibleIds]);

  useEffect(() => {
    onCountsChange?.({ visible: records.length, total: sourceRecords.length });
  }, [onCountsChange, records.length, sourceRecords.length]);

  const repoRoot = bootstrap?.repo_root ?? null;
  const multiSelectActive = multiSelection.size > 0;

  const onSelect = useCallback(
    (recordId: string) => {
      dispatch({ type: "SELECT_RECORD", payload: recordId });
    },
    [dispatch],
  );

  const onMultiSelectToggle = useCallback(
    (recordId: string, shiftKey: boolean) => {
      if (shiftKey && multiSelectActive) {
        dispatch({ type: "RANGE_MULTI_SELECT", payload: { targetId: recordId, visibleIds } });
      } else {
        dispatch({ type: "TOGGLE_MULTI_SELECT", payload: recordId });
      }
    },
    [dispatch, multiSelectActive, visibleIds],
  );

  const renderRow = useCallback(
    (record: RecordSummary) => (
      <RecordListItem
        key={record.record_id}
        recordId={record.record_id}
        record={record}
        repoRoot={repoRoot}
        isSelected={selectedRecordId === record.record_id}
        multiSelectActive={multiSelectActive}
        isMultiSelected={multiSelection.has(record.record_id)}
        onSelect={onSelect}
        onMultiSelectToggle={onMultiSelectToggle}
      />
    ),
    [multiSelectActive, multiSelection, onMultiSelectToggle, onSelect, repoRoot, selectedRecordId],
  );

  if (groups.length === 0) {
    return (
      <div className="flex flex-1 items-center justify-center">
        <p className="text-[11px] text-[var(--text-quaternary)]">No records</p>
      </div>
    );
  }

  return (
    <div className="min-h-0 flex-1 overflow-y-auto">
      {groups.map((group) => (
        <section key={group.key} className="border-b border-[var(--border-default)]">
          <div className="sticky top-0 z-10 flex flex-col gap-1.5 border-b border-[var(--border-subtle)] bg-[var(--surface-1)] px-3 py-2">
            <div className="flex items-center gap-2">
              <span className="text-[11px] font-semibold tracking-[0.01em] text-[var(--text-primary)]">
                {group.title}
              </span>
              <span className="text-[10px] text-[var(--text-tertiary)]">
                {group.originals.length + group.variants.length}
              </span>
            </div>
            {group.referenceImages.length > 0 ? (
              <div className="flex flex-wrap gap-1.5">
                {group.referenceImages.map((path) => (
                  <img
                    key={path}
                    src={pictureFileUrl(path)}
                    alt={group.title}
                    loading="lazy"
                    className="h-12 w-12 rounded border border-[var(--border-subtle)] object-cover"
                  />
                ))}
              </div>
            ) : null}
          </div>

          <div className="px-0.5 py-1">
            {group.originals.map(renderRow)}
            {group.variants.length > 0 ? (
              <>
                <p className="px-3 py-1 text-[9px] font-medium uppercase tracking-[0.08em] text-[var(--text-quaternary)]">
                  Variants
                </p>
                {group.variants.map(renderRow)}
              </>
            ) : null}
          </div>
        </section>
      ))}
    </div>
  );
}
