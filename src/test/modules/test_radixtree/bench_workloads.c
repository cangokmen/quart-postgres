/*--------------------------------------------------------------------------
 *
 * bench_workloads.c
 *		Standalone ART vs QuART insertion benchmark using BoDS workloads.
 *
 * Reads all workload_N5000000_*.bin files from a workload directory
 * (default: /home/grad1/cgokmen/bods/workloads/).  Each file contains
 * N 32-bit little-endian unsigned integer keys.  For every file the
 * program inserts all N keys into a plain ART tree and into a QuART
 * tree, reports wall-clock time for each, and spot-checks correctness.
 *
 * Self-contained: no running PostgreSQL instance is required.
 *
 * BUILD (from the repository root):
 *
 *   gcc -O2 -DUSE_NO_SIMD -I src/include \
 *       -o bench_workloads \
 *       src/test/modules/test_radixtree/bench_workloads.c
 *
 * RUN:
 *
 *   ./bench_workloads [workload_dir]
 *
 * Copyright (c) 2024-2026, PostgreSQL Global Development Group
 *
 * IDENTIFICATION
 *		src/test/modules/test_radixtree/bench_workloads.c
 *
 *--------------------------------------------------------------------------
 */

/*
 * ----------------------------------------------------------------
 * Step 1 – Pre-satisfy include guards for every PostgreSQL header
 * that radixtree.h / quart.h would otherwise pull in.
 *
 * When the preprocessor later encounters, e.g.,
 *
 *		#include "nodes/bitmapset.h"
 *
 * inside radixtree.h, the header is opened but its body is skipped
 * because we already defined the guard.  We then supply the handful
 * of symbols that radixtree.h actually uses ourselves (Sections 2-5).
 * ----------------------------------------------------------------
 */
#define POSTGRES_H			/* replaced by Section 2 below          */
#define BITMAPSET_H			/* replaced by Section 3 below          */
#define PG_BITUTILS_H		/* replaced by Section 4 below          */
#define SIMD_H				/* replaced by Section 5 below (NO_SIMD)*/
#define MEMUTILS_H			/* replaced by Section 6 below          */
#define DSA_H				/* only needed for RT_SHMEM             */
#define MISCADMIN_H			/* nothing used in non-SHMEM mode       */
#define LWLOCK_H			/* only needed for RT_SHMEM             */

/* ----------------------------------------------------------------
 * Section 1 – Standard library (safe to include unconditionally)
 * ---------------------------------------------------------------- */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <errno.h>
#include <glob.h>
#include <time.h>

/* ----------------------------------------------------------------
 * Section 2 – Replace postgres.h / c.h
 *
 * Provides the types, constants, and compiler-attribute macros that
 * radixtree.h relies on from the PostgreSQL base headers.
 * ---------------------------------------------------------------- */

/* --- Basic integer types (mirrors c.h) --- */
typedef uint8_t		uint8;
typedef uint16_t	uint16;
typedef uint32_t	uint32;
typedef uint64_t	uint64;
typedef int8_t		int8;
typedef int16_t		int16;
typedef int32_t		int32;
typedef int64_t		int64;
typedef size_t		Size;

/* --- Numeric constants --- */
#define UINT64CONST(x)		((uint64)(UINT64_C(x)))
#define INT64CONST(x)		((int64)(INT64_C(x)))
#define BITS_PER_BYTE		8

#ifndef PG_UINT32_MAX
#define PG_UINT32_MAX		UINT32_MAX
#endif
#ifndef PG_UINT64_MAX
#define PG_UINT64_MAX		UINT64_MAX
#endif

/* bits8 – used only in pg_bitutils.h declarations we'll never call */
typedef uint8 bits8;

/* sizeof-based configure constants needed by pg_bitutils.h */
#define SIZEOF_LONG			__SIZEOF_LONG__
#define SIZEOF_LONG_LONG	__SIZEOF_LONG_LONG__
#define SIZEOF_SIZE_T		__SIZEOF_SIZE_T__

/* --- Compiler / attribute macros --- */
#define pg_attribute_packed()		__attribute__((packed))
#define pg_attribute_aligned(n)		__attribute__((aligned(n)))
#define pg_unreachable()			__builtin_unreachable()
#define pg_attribute_unused()		__attribute__((unused))
#define pg_noinline					__attribute__((noinline))
#define pg_attribute_noreturn()		__attribute__((noreturn))
#define PGDLLIMPORT
#define likely(x)					__builtin_expect((x) != 0, 1)
#define unlikely(x)					__builtin_expect((x) != 0, 0)

/*
 * Assert is a deliberate no-op here (matching a non-debug PG build).
 * It must NOT evaluate its argument: some variables are declared only
 * inside #ifdef USE_ASSERT_CHECKING blocks in simd.h.
 */
#define Assert(x)					((void)(true))
#define AssertArg(x)				((void)(true))
#define AssertState(x)				((void)(true))
#define StaticAssertDecl(c, m)		_Static_assert(c, m)
#define StaticAssertStmt(c, m)		_Static_assert(c, m)
#define PG_USED_FOR_ASSERTS_ONLY	pg_attribute_unused()

/* --- Container helpers --- */
#define FLEXIBLE_ARRAY_MEMBER		/* empty */
#define lengthof(a)					(sizeof(a) / sizeof((a)[0]))
#ifndef Max
#define Max(a, b)	((a) > (b) ? (a) : (b))
#endif
#ifndef Min
#define Min(a, b)	((a) < (b) ? (a) : (b))
#endif

/* Token concatenation used by radixtree.h symbol generation */
#define CppConcat(x, y)		x##y

/* ----------------------------------------------------------------
 * Section 3 – Replace nodes/bitmapset.h
 *
 * Provides bitmapword, BITS_PER_BITMAPWORD, and bmw_rightmost_one_pos.
 * The real header is guarded by BITMAPSET_H (defined above).
 * ---------------------------------------------------------------- */
#if defined(__LP64__) || defined(_LP64) || defined(__x86_64__)
# define BITS_PER_BITMAPWORD	64
typedef uint64	bitmapword;
typedef int64	signedbitmapword;
#else
# define BITS_PER_BITMAPWORD	32
typedef uint32	bitmapword;
typedef int32	signedbitmapword;
#endif

/* ----------------------------------------------------------------
 * Section 4 – Replace port/pg_bitutils.h
 *
 * Only the three functions actually called by radixtree.h are
 * implemented here:
 *   pg_leftmost_one_pos64  – used to compute the shift for a key
 *   pg_rightmost_one_pos64 – used for bitmapword bit-scan (node48)
 *   pg_nextpower2_32       – used for slab block size (value unused
 *                            in our arena allocator)
 *
 * pg_rightmost_one_pos32 is only used inside the SIMD path
 * (#ifndef USE_NO_SIMD), so not needed here.
 * ---------------------------------------------------------------- */
static inline int
pg_leftmost_one_pos64(uint64 word)
{
	return 63 - __builtin_clzll((unsigned long long) word);
}

static inline int
pg_rightmost_one_pos64(uint64 word)
{
	return __builtin_ctzll((unsigned long long) word);
}

static inline int
pg_rightmost_one_pos32(uint32 word)
{
	return __builtin_ctz((unsigned int) word);
}

static inline uint32
pg_nextpower2_32(uint32 num)
{
	if (num == 0)
		return 1;
	if ((num & (num - 1)) == 0)
		return num;				/* already a power of 2 */
	return (uint32) 1u << (32 - __builtin_clz((unsigned int) num));
}

/*
 * bmw_rightmost_one_pos – position of the rightmost set bit in a
 * bitmapword (0-based from LSB).
 */
#if BITS_PER_BITMAPWORD == 64
#define bmw_rightmost_one_pos(w)	pg_rightmost_one_pos64(w)
#else
#define bmw_rightmost_one_pos(w)	pg_rightmost_one_pos32(w)
#endif

/* ----------------------------------------------------------------
 * Section 5 – Replace port/simd.h
 *
 * Defining USE_NO_SIMD makes radixtree.h use plain scalar loops for
 * node-16 search.  All vector8_* function calls live inside
 * "#ifndef USE_NO_SIMD" guards and are never compiled.  We still
 * need Vector8 as a type because its sizeof() is used to compute
 * the load stride, but only inside those guarded blocks.
 * ---------------------------------------------------------------- */
#ifndef USE_NO_SIMD
#define USE_NO_SIMD
#endif

typedef uint64	Vector8;

/* ----------------------------------------------------------------
 * Section 6 – Replace utils/memutils.h
 *
 * A simple arena allocator that satisfies the MemoryContext API
 * radixtree.h uses in non-SHMEM mode:
 *
 *   AllocSetContextCreate / SlabContextCreate
 *       → allocate and link a child context
 *   MemoryContextAlloc
 *       → bump-pointer allocation from 64 MB blocks
 *   MemoryContextReset
 *       → recursively free all child contexts and all blocks
 *   MemoryContextDelete
 *       → reset then free the context struct itself
 *   palloc / palloc0 / palloc0_object / pfree
 *       → helpers routed through CurrentMemoryContext
 *         (pfree is a no-op; the arena is freed all at once)
 * ---------------------------------------------------------------- */
#define ARENA_BLOCK_SIZE	(64UL * 1024 * 1024)

typedef struct ArenaBlock		ArenaBlock;
typedef struct MemoryContextData MemoryContextData;
typedef MemoryContextData		*MemoryContext;

struct ArenaBlock
{
	ArenaBlock *next;
	size_t		used;
	char		data[ARENA_BLOCK_SIZE];
};

struct MemoryContextData
{
	ArenaBlock		   *head;
	MemoryContextData  *first_child;
	MemoryContextData  *next_sibling;
	MemoryContextData  *parent;
};

static inline MemoryContext
mcxt_new_child(MemoryContext parent)
{
	MemoryContextData *ctx = (MemoryContextData *) calloc(1, sizeof(*ctx));

	if (!ctx)
	{
		perror("calloc"); abort();
	}
	if (parent)
	{
		ctx->parent = parent;
		ctx->next_sibling = parent->first_child;
		parent->first_child = ctx;
	}
	return ctx;
}

static inline void *
MemoryContextAlloc(MemoryContext ctx, size_t size)
{
	size = (size + 7) & ~(size_t) 7;	/* 8-byte align */

	if (!ctx->head || ctx->head->used + size > ARENA_BLOCK_SIZE)
	{
		ArenaBlock *blk = (ArenaBlock *) calloc(1, sizeof(ArenaBlock));

		if (!blk)
		{
			perror("calloc"); abort();
		}
		blk->next = ctx->head;
		ctx->head = blk;
	}
	void	   *ptr = ctx->head->data + ctx->head->used;

	ctx->head->used += size;
	return ptr;
}

static inline void
MemoryContextReset(MemoryContext ctx)
{
	/* Recursively delete child contexts */
	MemoryContextData *child = ctx->first_child;

	while (child)
	{
		MemoryContextData *next = child->next_sibling;
		ArenaBlock *blk;

		MemoryContextReset(child);
		blk = child->head;
		while (blk)
		{
			ArenaBlock *n = blk->next;

			free(blk);
			blk = n;
		}
		free(child);
		child = next;
	}
	ctx->first_child = NULL;

	/* Free this context's own blocks */
	{
		ArenaBlock *blk = ctx->head;

		while (blk)
		{
			ArenaBlock *n = blk->next;

			free(blk);
			blk = n;
		}
		ctx->head = NULL;
	}
}

static inline void
MemoryContextDelete(MemoryContext ctx)
{
	MemoryContextReset(ctx);
	free(ctx);
}

#define MemoryContextMemAllocated(ctx, r)	((Size) 0)

/* Global context used by palloc() in non-tree code */
static MemoryContextData bench_top_ctx;
static MemoryContext CurrentMemoryContext = &bench_top_ctx;

#define palloc(sz)			MemoryContextAlloc(CurrentMemoryContext, (sz))
#define palloc0(sz)			MemoryContextAlloc(CurrentMemoryContext, (sz))
#define pfree(p)			((void)(p))
#define palloc0_object(T)	((T *) MemoryContextAlloc(CurrentMemoryContext, sizeof(T)))
#define palloc_object(T)	((T *) MemoryContextAlloc(CurrentMemoryContext, sizeof(T)))
#define palloc_array(T, n)	((T *) MemoryContextAlloc(CurrentMemoryContext, sizeof(T) * (n)))

static inline MemoryContext
AllocSetContextCreateInternal(MemoryContext parent, const char *name, ...)
{
	(void) name;
	return mcxt_new_child(parent);
}

/* Strip size-hint arguments: callers pass ALLOCSET_DEFAULT_SIZES etc. */
#define AllocSetContextCreate(parent, name, ...) \
	AllocSetContextCreateInternal((parent), (name))

#define ALLOCSET_DEFAULT_SIZES	0, 0, 0
#define ALLOCSET_SMALL_SIZES	0, 0, 0
#define SLAB_DEFAULT_BLOCK_SIZE (8 * 1024)

static inline MemoryContext
SlabContextCreate(MemoryContext parent, const char *name,
				  size_t blockSize, size_t chunkSize)
{
	(void) name;
	(void) blockSize;
	(void) chunkSize;
	return mcxt_new_child(parent);
}

/* ================================================================
 * Section 7 – Instantiate ART and QuART trees
 *
 * The include-guard trick (Sections 1-6) ensures that when
 * radixtree.h includes "nodes/bitmapset.h", "port/pg_bitutils.h",
 * etc., those headers are opened but their bodies are skipped
 * because the guards are already defined above.
 * ================================================================ */

/* Plain ART – used as the baseline */
#define RT_PREFIX		art
#define RT_SCOPE		static
#define RT_DECLARE
#define RT_DEFINE
#define RT_VALUE_TYPE	uint64
#include "lib/radixtree.h"

/* QuART – uses the fast-path optimisation from quart.h */
#define RT_PREFIX		quart
#define RT_SCOPE		static
#define RT_DECLARE
#define RT_DEFINE
#define RT_USE_QUART
#define RT_VALUE_TYPE	uint64
#include "lib/radixtree.h"

/* ================================================================
 * Section 8 – Benchmark driver
 * ================================================================ */

static double
elapsed_ms(struct timespec start, struct timespec end)
{
	return (double) (end.tv_sec - start.tv_sec) * 1000.0
		+ (double) (end.tv_nsec - start.tv_nsec) / 1e6;
}

/*
 * Load N 32-bit little-endian keys from a binary workload file.
 * Returns a malloc'd array of uint64 (widened keys) and sets *out_n,
 * or returns NULL on error.
 */
static uint64 *
load_workload(const char *path, size_t *out_n)
{
	FILE	   *f = fopen(path, "rb");

	if (!f)
	{
		fprintf(stderr, "open %s: %s\n", path, strerror(errno));
		return NULL;
	}

	if (fseek(f, 0, SEEK_END))
	{
		perror("fseek"); fclose(f); return NULL;
	}
	long		fsz = ftell(f);

	rewind(f);

	if (fsz <= 0 || fsz % 4)
	{
		fprintf(stderr, "%s: unexpected file size %ld\n", path, fsz);
		fclose(f);
		return NULL;
	}

	size_t		n = (size_t) fsz / 4;
	uint32	   *raw = (uint32 *) malloc(sizeof(uint32) * n);

	if (!raw)
	{
		perror("malloc"); fclose(f); return NULL;
	}
	if (fread(raw, 4, n, f) != n)
	{
		fprintf(stderr, "%s: short read\n", path);
		free(raw); fclose(f); return NULL;
	}
	fclose(f);

	uint64	   *keys = (uint64 *) malloc(sizeof(uint64) * n);

	if (!keys)
	{
		perror("malloc"); free(raw); return NULL;
	}
	for (size_t i = 0; i < n; i++)
		keys[i] = (uint64) raw[i];

	free(raw);
	*out_n = n;
	return keys;
}

#define BENCH_RUNS		20

/*
 * Run one timed trial: insert all N keys into ART and QuART, then
 * look up all N keys in each tree.  On the first trial (run == 0)
 * also verify correctness of every lookup result.
 * Returns true on success, false if verification failed.
 */
static bool
run_one_trial(const uint64 *keys, size_t n, int run,
			  double *out_art_insert_ms, double *out_quart_insert_ms,
			  double *out_art_lookup_ms, double *out_quart_lookup_ms)
{
	struct timespec t0,
				t1;

	MemoryContext art_ctx = AllocSetContextCreate(CurrentMemoryContext,
												  "art", ALLOCSET_DEFAULT_SIZES);
	MemoryContext quart_ctx = AllocSetContextCreate(CurrentMemoryContext,
													"quart", ALLOCSET_DEFAULT_SIZES);

	art_radix_tree *art_tree = art_create(art_ctx);
	quart_radix_tree *quart_tree = quart_create(quart_ctx);

	/* --- ART insert --- */
	clock_gettime(CLOCK_MONOTONIC, &t0);
	for (size_t i = 0; i < n; i++)
	{
		uint64		val = keys[i];

		art_set(art_tree, keys[i], &val);
	}
	clock_gettime(CLOCK_MONOTONIC, &t1);
	*out_art_insert_ms = elapsed_ms(t0, t1);

	/* --- QuART insert --- */
	clock_gettime(CLOCK_MONOTONIC, &t0);
	for (size_t i = 0; i < n; i++)
	{
		uint64		val = keys[i];

		quart_set_quart(quart_tree, keys[i], &val);
	}
	clock_gettime(CLOCK_MONOTONIC, &t1);
	*out_quart_insert_ms = elapsed_ms(t0, t1);

	/* --- ART lookup (all N keys) --- */
	clock_gettime(CLOCK_MONOTONIC, &t0);
	for (size_t i = 0; i < n; i++)
		(void) art_find(art_tree, keys[i]);
	clock_gettime(CLOCK_MONOTONIC, &t1);
	*out_art_lookup_ms = elapsed_ms(t0, t1);

	/* --- QuART lookup (all N keys) --- */
	clock_gettime(CLOCK_MONOTONIC, &t0);
	for (size_t i = 0; i < n; i++)
		(void) quart_find(quart_tree, keys[i]);
	clock_gettime(CLOCK_MONOTONIC, &t1);
	*out_quart_lookup_ms = elapsed_ms(t0, t1);

	/* Verify correctness on first run only */
	bool		ok = true;

	if (run == 0)
	{
		for (size_t i = 0; i < n; i++)
		{
			uint64	   *av = art_find(art_tree, keys[i]);
			uint64	   *qv = quart_find(quart_tree, keys[i]);

			if (!av || *av != keys[i])
			{
				fprintf(stderr,
						"ART verification failed at index %zu (key=%" PRIu64 ")\n",
						i, keys[i]);
				ok = false;
				break;
			}
			if (!qv || *qv != keys[i])
			{
				fprintf(stderr,
						"QuART verification failed at index %zu (key=%" PRIu64 ")\n",
						i, keys[i]);
				ok = false;
				break;
			}
		}
	}

	art_free(art_tree);
	quart_free(quart_tree);
	MemoryContextDelete(art_ctx);
	MemoryContextDelete(quart_ctx);

	return ok;
}

/*
 * Run BENCH_RUNS trials for one workload, printing per-trial lines to
 * stdout and appending a summary to the results file.
 */
static void
run_workload(const char *label, const uint64 *keys, size_t n, FILE *results, FILE *csv)
{
	double		art_insert_ms[BENCH_RUNS];
	double		quart_insert_ms[BENCH_RUNS];
	double		art_lookup_ms[BENCH_RUNS];
	double		quart_lookup_ms[BENCH_RUNS];

	printf("=== %s (N=%zu, %d runs) ===\n", label, n, BENCH_RUNS);
	fprintf(results, "=== %s (N=%zu, %d runs) ===\n", label, n, BENCH_RUNS);

	for (int r = 0; r < BENCH_RUNS; r++)
	{
		if (!run_one_trial(keys, n, r,
						   &art_insert_ms[r], &quart_insert_ms[r],
						   &art_lookup_ms[r], &quart_lookup_ms[r]))
		{
			printf("  run %d: VERIFICATION FAILED\n", r + 1);
			fprintf(results, "  run %d: VERIFICATION FAILED\n", r + 1);
			fflush(stdout);
			fflush(results);
			return;
		}

		double		ins_ratio = quart_insert_ms[r] > 0 ? art_insert_ms[r] / quart_insert_ms[r] : 0.0;
		double		lkp_ratio = quart_lookup_ms[r] > 0 ? art_lookup_ms[r] / quart_lookup_ms[r] : 0.0;

		printf("  run %d: insert ART %8.2f ms  QuART %8.2f ms  ratio %.2fx  "
			   "lookup ART %8.2f ms  QuART %8.2f ms  ratio %.2fx\n",
			   r + 1,
			   art_insert_ms[r], quart_insert_ms[r], ins_ratio,
			   art_lookup_ms[r], quart_lookup_ms[r], lkp_ratio);
		fprintf(results,
				"  run %d: insert ART %8.2f ms  QuART %8.2f ms  ratio %.2fx  "
				"lookup ART %8.2f ms  QuART %8.2f ms  ratio %.2fx\n",
				r + 1,
				art_insert_ms[r], quart_insert_ms[r], ins_ratio,
				art_lookup_ms[r], quart_lookup_ms[r], lkp_ratio);
		fflush(stdout);
		fflush(results);
	}

	/* Compute min / mean across trials */
	double		art_ins_sum = 0,
				quart_ins_sum = 0;
	double		art_lkp_sum = 0,
				quart_lkp_sum = 0;
	double		art_ins_min = art_insert_ms[0],
				quart_ins_min = quart_insert_ms[0];
	double		art_lkp_min = art_lookup_ms[0],
				quart_lkp_min = quart_lookup_ms[0];

	for (int r = 0; r < BENCH_RUNS; r++)
	{
		art_ins_sum += art_insert_ms[r];
		quart_ins_sum += quart_insert_ms[r];
		art_lkp_sum += art_lookup_ms[r];
		quart_lkp_sum += quart_lookup_ms[r];
		if (art_insert_ms[r] < art_ins_min)
			art_ins_min = art_insert_ms[r];
		if (quart_insert_ms[r] < quart_ins_min)
			quart_ins_min = quart_insert_ms[r];
		if (art_lookup_ms[r] < art_lkp_min)
			art_lkp_min = art_lookup_ms[r];
		if (quart_lookup_ms[r] < quart_lkp_min)
			quart_lkp_min = quart_lookup_ms[r];
	}
	double		art_ins_mean = art_ins_sum / BENCH_RUNS;
	double		quart_ins_mean = quart_ins_sum / BENCH_RUNS;
	double		art_lkp_mean = art_lkp_sum / BENCH_RUNS;
	double		quart_lkp_mean = quart_lkp_sum / BENCH_RUNS;

	printf("  INSERT SUMMARY:  ART mean %8.2f ms (min %8.2f ms)  "
		   "QuART mean %8.2f ms (min %8.2f ms)  "
		   "mean ratio %.2fx  best ratio %.2fx\n"
		   "  LOOKUP SUMMARY:  ART mean %8.2f ms (min %8.2f ms)  "
		   "QuART mean %8.2f ms (min %8.2f ms)  "
		   "mean ratio %.2fx  best ratio %.2fx\n\n",
		   art_ins_mean, art_ins_min, quart_ins_mean, quart_ins_min,
		   quart_ins_mean > 0 ? art_ins_mean / quart_ins_mean : 0.0,
		   quart_ins_min > 0 ? art_ins_min / quart_ins_min : 0.0,
		   art_lkp_mean, art_lkp_min, quart_lkp_mean, quart_lkp_min,
		   quart_lkp_mean > 0 ? art_lkp_mean / quart_lkp_mean : 0.0,
		   quart_lkp_min > 0 ? art_lkp_min / quart_lkp_min : 0.0);
	fprintf(results,
			"  INSERT SUMMARY:  ART mean %8.2f ms (min %8.2f ms)  "
			"QuART mean %8.2f ms (min %8.2f ms)  "
			"mean ratio %.2fx  best ratio %.2fx\n"
			"  LOOKUP SUMMARY:  ART mean %8.2f ms (min %8.2f ms)  "
			"QuART mean %8.2f ms (min %8.2f ms)  "
			"mean ratio %.2fx  best ratio %.2fx\n\n",
			art_ins_mean, art_ins_min, quart_ins_mean, quart_ins_min,
			quart_ins_mean > 0 ? art_ins_mean / quart_ins_mean : 0.0,
			quart_ins_min > 0 ? art_ins_min / quart_ins_min : 0.0,
			art_lkp_mean, art_lkp_min, quart_lkp_mean, quart_lkp_min,
			quart_lkp_mean > 0 ? art_lkp_mean / quart_lkp_mean : 0.0,
			quart_lkp_min > 0 ? art_lkp_min / quart_lkp_min : 0.0);

	/* Parse K and L values from filename (workload_N<n>_K<k>_L<l>.bin) */
	char		k_str[64] = "?",
				l_str[64] = "?";

	sscanf(label, "workload_N%*[^_]_K%63[^_]_L%63[^.]", k_str, l_str);

	/*
	 * Convert encoded K/L strings to decimal fractions by inserting a
	 * decimal point after the first digit: "001"->0.01, "01"->0.1, "05"->0.5.
	 */
	auto double decode_kl(const char *s);
	double decode_kl(const char *s)
	{
		size_t		len = strlen(s);
		char		buf[32];

		if (len <= 1)
			return strtod(s, NULL);
		buf[0] = s[0];
		buf[1] = '.';
		memcpy(buf + 2, s + 1, len);	/* includes NUL */
		return strtod(buf, NULL);
	}

	double		k_val = decode_kl(k_str);
	double		l_val = decode_kl(l_str);

	/* Write one row per tree type: tree_type,N,K,L,repeat,avg_insert_ms */
	fprintf(csv, "ART,%zu,%.3g,%.3g,%d,%.2f\n",
			n, k_val, l_val, BENCH_RUNS, art_ins_mean);
	fprintf(csv, "QuART,%zu,%.3g,%.3g,%d,%.2f\n",
			n, k_val, l_val, BENCH_RUNS, quart_ins_mean);
	fflush(csv);

	fflush(stdout);
	fflush(results);
}

int
main(int argc, char *argv[])
{
	const char *dir = (argc > 1) ? argv[1]
		: "/home/grad1/cgokmen/bods/workloads";

	/* Results file lives next to the binary / cwd */
	const char *results_path = "bench_results.txt";
	FILE	   *results = fopen(results_path, "w");

	if (!results)
	{
		perror(results_path);
		return 1;
	}

	const char *csv_path = "bench_results.csv";
	FILE	   *csv = fopen(csv_path, "w");

	if (!csv)
	{
		perror(csv_path);
		fclose(results);
		return 1;
	}
	fprintf(csv, "tree_type,N,K,L,repeat,avg_insert_ms\n");

	char		pattern[1024];

	snprintf(pattern, sizeof(pattern),
			 "%s/workload_N5000000_*.bin", dir);

	glob_t		gl;
	int			rc = glob(pattern, GLOB_NOSORT, NULL, &gl);

	if (rc == GLOB_NOMATCH)
	{
		fprintf(stderr, "No workload files matched: %s\n", pattern);
		fclose(results);
		return 1;
	}
	if (rc)
	{
		perror("glob");
		fclose(results);
		return 1;
	}

	/* Print header to both stdout and file */
	char		header[256];

	snprintf(header, sizeof(header),
			 "ART vs QuART insertion benchmark — %d runs per workload\n"
			 "Workload directory: %s\n"
			 "Found %zu file(s)\n\n",
			 BENCH_RUNS, dir, gl.gl_pathc);
	printf("%s", header);
	fprintf(results, "%s", header);

	for (size_t fi = 0; fi < gl.gl_pathc; fi++)
	{
		const char *path = gl.gl_pathv[fi];
		const char *label = strrchr(path, '/');

		label = label ? label + 1 : path;

		size_t		n;
		uint64	   *keys = load_workload(path, &n);

		if (!keys)
			continue;

		run_workload(label, keys, n, results, csv);
		free(keys);
	}

	globfree(&gl);
	fclose(results);
	fclose(csv);
	printf("Results written to %s\n", results_path);
	printf("CSV averages written to %s\n", csv_path);
	return 0;
}
