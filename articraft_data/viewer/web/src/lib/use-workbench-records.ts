import { useDeferredValue, useEffect, useMemo, useState } from "react";

import { useQueryClient } from "@tanstack/react-query";

import { recordMatchesAgentHarnessFilters } from "@/lib/agent-harness";
import type { CostFilter, RatingFilter, RecordSummary, TimeFilter } from "@/lib/types";
import { useViewer, useViewerDispatch } from "@/lib/viewer-context";
import { searchRecordsQueryOptions } from "@/lib/viewer-queries";

// NOTE: these mirror the workbench filtering pipeline in
// components/browser/RecordList.tsx so the flat (virtualized) list and the
// grouped-by-subcategory list stay consistent. Keep the two in sync if the
// workbench filter semantics change.

const TIME_DURATIONS: Record<string, number> = {
  "1h": 1 * 60 * 60 * 1000,
  "6h": 6 * 60 * 60 * 1000,
  "12h": 12 * 60 * 60 * 1000,
  "24h": 24 * 60 * 60 * 1000,
  "3d": 3 * 24 * 60 * 60 * 1000,
  "7d": 7 * 24 * 60 * 60 * 1000,
  "14d": 14 * 24 * 60 * 60 * 1000,
  "30d": 30 * 24 * 60 * 60 * 1000,
  "60d": 60 * 24 * 60 * 60 * 1000,
  "90d": 90 * 24 * 60 * 60 * 1000,
  "180d": 180 * 24 * 60 * 60 * 1000,
  "1y": 365 * 24 * 60 * 60 * 1000,
};

function withinTimeFilter(createdAt: string | null, filter: TimeFilter): boolean {
  if (!filter.oldest && !filter.newest) return true;
  if (!createdAt) return false;

  const createdAtMs = new Date(createdAt).getTime();
  if (Number.isNaN(createdAtMs)) return false;

  const age = Date.now() - createdAtMs;
  if (filter.oldest) {
    const maxAge = TIME_DURATIONS[filter.oldest];
    if (maxAge != null && age > maxAge) return false;
  }
  if (filter.newest) {
    const minAge = TIME_DURATIONS[filter.newest];
    if (minAge != null && age < minAge) return false;
  }
  return true;
}

function withinCostFilter(totalCostUsd: number | null, filter: CostFilter): boolean {
  if (filter.min == null && filter.max == null) return true;
  if (totalCostUsd == null) return false;
  if (filter.min != null && totalCostUsd < filter.min) return false;
  if (filter.max != null && totalCostUsd > filter.max) return false;
  return true;
}

function withinRatingFilter(rating: number | null, filter: RatingFilter): boolean {
  if (filter.length === 0) return true;
  if (rating == null) return filter.includes("unrated");
  if (rating < 2) return filter.includes("1");
  if (rating < 3) return filter.includes("2");
  if (rating < 4) return filter.includes("3");
  if (rating < 5) return filter.includes("4");
  return filter.includes("5");
}

function recordSortTimestamp(record: RecordSummary): number {
  const timestamp = record.updated_at ?? record.created_at;
  return timestamp ? new Date(timestamp).getTime() : 0;
}

function subcategoryKeyForRecord(record: RecordSummary): string | null {
  if (!record.picture_category || !record.picture_subcategory) {
    return null;
  }
  return `${record.picture_category}/${record.picture_subcategory}`;
}

function withinSubcategoryFilter(record: RecordSummary, subcategoryFilters: string[]): boolean {
  if (subcategoryFilters.length === 0) {
    return true;
  }
  const key = subcategoryKeyForRecord(record);
  return key != null && subcategoryFilters.includes(key);
}

export type WorkbenchRecords = {
  records: RecordSummary[];
  sourceRecords: RecordSummary[];
  searchPending: boolean;
};

export function useWorkbenchRecords(): WorkbenchRecords {
  const queryClient = useQueryClient();
  const {
    bootstrap,
    searchQuery,
    sourceFilter,
    timeFilter,
    modelFilter,
    sdkFilter,
    agentHarnessFilters,
    costFilter,
    ratingFilter,
    secondaryRatingFilter,
    subcategoryFilters,
    selectedRunId,
  } = useViewer();
  const dispatch = useViewerDispatch();
  const deferredSearchQuery = useDeferredValue(searchQuery.trim());
  const [searchedRecords, setSearchedRecords] = useState<RecordSummary[] | null>(null);
  const [searchPending, setSearchPending] = useState(false);

  const sourceRecords = useMemo(() => {
    if (!bootstrap || sourceFilter !== "workbench") {
      return [];
    }
    const seen = new Map<string, RecordSummary>();
    for (const entry of bootstrap.workbench_entries) {
      if (entry.record && !seen.has(entry.record_id)) {
        seen.set(entry.record_id, entry.record);
      }
    }
    return Array.from(seen.values());
  }, [bootstrap, sourceFilter]);

  const recordById = useMemo(
    () => new Map(sourceRecords.map((record) => [record.record_id, record])),
    [sourceRecords],
  );

  useEffect(() => {
    if (sourceFilter !== "workbench" || !deferredSearchQuery) {
      setSearchedRecords(null);
      setSearchPending(false);
      return;
    }

    let cancelled = false;
    setSearchedRecords(null);
    setSearchPending(true);

    queryClient
      .fetchQuery(
        searchRecordsQueryOptions({
          query: deferredSearchQuery,
          source: sourceFilter,
          runId: selectedRunId,
          timeFilter,
          modelFilter,
          sdkFilter,
          agentHarnessFilters,
          authorFilters: [],
          categoryFilters: [],
          costFilter,
          ratingFilter,
          secondaryRatingFilter,
          limit: 200,
        }),
      )
      .then((results) => {
        if (!cancelled) {
          setSearchedRecords(results);
          setSearchPending(false);
          dispatch({ type: "UPSERT_RECORDS", payload: results });
        }
      })
      .catch(() => {
        if (!cancelled) {
          setSearchedRecords([]);
          setSearchPending(false);
        }
      });

    return () => {
      cancelled = true;
    };
  }, [
    costFilter,
    deferredSearchQuery,
    dispatch,
    modelFilter,
    agentHarnessFilters,
    ratingFilter,
    secondaryRatingFilter,
    sdkFilter,
    selectedRunId,
    sourceFilter,
    timeFilter,
    queryClient,
  ]);

  const records = useMemo(() => {
    if (!bootstrap || sourceFilter !== "workbench") return [];

    let list = deferredSearchQuery
      ? (searchedRecords ?? []).map((record) => recordById.get(record.record_id) ?? record)
      : sourceRecords;

    if (!deferredSearchQuery && selectedRunId) {
      list = list.filter((record) => record.run_id === selectedRunId);
    }

    list = list.filter((record) => withinSubcategoryFilter(record, subcategoryFilters));

    if (!deferredSearchQuery) {
      list = list.filter((record) => withinTimeFilter(record.created_at, timeFilter));
      list = list.filter((record) => withinCostFilter(record.total_cost_usd, costFilter));
      list = list.filter((record) => withinRatingFilter(record.rating, ratingFilter));
      list = list.filter((record) =>
        withinRatingFilter(record.secondary_rating ?? null, secondaryRatingFilter),
      );
      if (modelFilter) {
        list = list.filter((record) => record.model_id === modelFilter);
      }
      if (sdkFilter) {
        list = list.filter((record) => record.sdk_package === sdkFilter);
      }
      list = list.filter((record) => recordMatchesAgentHarnessFilters(record, agentHarnessFilters));
      list = [...list].sort((left, right) => recordSortTimestamp(right) - recordSortTimestamp(left));
    }

    return list;
  }, [
    bootstrap,
    costFilter,
    deferredSearchQuery,
    modelFilter,
    agentHarnessFilters,
    ratingFilter,
    secondaryRatingFilter,
    searchedRecords,
    sdkFilter,
    selectedRunId,
    sourceFilter,
    subcategoryFilters,
    timeFilter,
    recordById,
    sourceRecords,
  ]);

  return { records, sourceRecords, searchPending };
}
