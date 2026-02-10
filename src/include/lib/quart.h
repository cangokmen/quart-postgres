/*-------------------------------------------------------------------------
 *
 * quart.h
 *		QuART (Quick Adaptive Radix Tree) extension for radixtree.h
 *
 * This file contains the QuART stail_reset_bidir optimization implementation
 * for the adaptive radix tree template. It should be included by radixtree.h
 * when RT_USE_QUART is defined.
 *
 * The QuART optimization implements:
 * - Byte-by-byte comparison of first 3 bytes against fp_leaf_value
 * - Bidirectional pattern detection (ascending/descending)
 * - Bridge value handling for byte boundary crossings
 * - Reset counter (300 iterations) for non-sequential inserts
 * - Fast path insert when first 3 bytes match fp_leaf_value
 *
 * Note: The RT_QUART_CONTROL_FIELDS and RT_QUART_INIT macros are defined
 * in radixtree.h before the structure definitions.
 *
 * Copyright (c) 2024-2026, PostgreSQL Global Development Group
 *
 * IDENTIFICATION
 *	  src/include/lib/quart.h
 *
 *-------------------------------------------------------------------------
 */

#ifndef RT_USE_QUART
#error "quart.h should only be included when RT_USE_QUART is defined"
#endif

/*
 * Node-specific insertion functions that update fast path (change_fp variants)
 */

/* Add child to Node4 and update fast path */
static inline void
RT_QUART_ADD_CHILD_4_CHANGE_FP(RT_RADIX_TREE *tree, RT_PTR_ALLOC *parent_slot,
							   RT_CHILD_PTR node, uint8 chunk, RT_PTR_ALLOC child)
{
	RT_NODE_4 *n4 = (RT_NODE_4 *) node.local;
	int count = n4->base.count;
	int insertpos = RT_NODE_4_GET_INSERTPOS(n4, chunk, count);

	/* shift chunks and children */
	RT_SHIFT_ARRAYS_FOR_INSERT(n4->chunks, n4->children, count, insertpos);

	/* insert new chunk and child */
	n4->chunks[insertpos] = chunk;
	n4->children[insertpos] = child;
	n4->base.count++;

	/* Update fast path */
	tree->ctl->fp = node.alloc;
	tree->ctl->fp_ref = parent_slot;
	tree->ctl->fp_leaf = child;
	tree->ctl->fp_shift = 0;  /* Parent of leaves is at shift 0 */

	RT_VERIFY_NODE((RT_NODE *) n4);
}

/* Add child to Node16 and update fast path */
static inline void
RT_QUART_ADD_CHILD_16_CHANGE_FP(RT_RADIX_TREE *tree, RT_PTR_ALLOC *parent_slot,
								RT_CHILD_PTR node, uint8 chunk, RT_PTR_ALLOC child)
{
	RT_NODE_16 *n16 = (RT_NODE_16 *) node.local;
	int insertpos = RT_NODE_16_GET_INSERTPOS(n16, chunk);

	/* shift chunks and children */
	RT_SHIFT_ARRAYS_FOR_INSERT(n16->chunks, n16->children, n16->base.count, insertpos);

	/* insert new chunk and child */
	n16->chunks[insertpos] = chunk;
	n16->children[insertpos] = child;
	n16->base.count++;

	/* Update fast path */
	tree->ctl->fp = node.alloc;
	tree->ctl->fp_ref = parent_slot;
	tree->ctl->fp_leaf = child;
	tree->ctl->fp_shift = 0;  /* Parent of leaves is at shift 0 */

	RT_VERIFY_NODE((RT_NODE *) n16);
}

/* Add child to Node48 and update fast path */
static inline void
RT_QUART_ADD_CHILD_48_CHANGE_FP(RT_RADIX_TREE *tree, RT_PTR_ALLOC *parent_slot,
								RT_CHILD_PTR node, uint8 chunk, RT_PTR_ALLOC child)
{
	RT_NODE_48 *n48 = (RT_NODE_48 *) node.local;
	int insertpos = n48->base.count;
	int idx = RT_BM_IDX(insertpos);
	int bitnum = RT_BM_BIT(insertpos);

	/* mark the slot used */
	n48->isset[idx] |= ((bitmapword) 1 << bitnum);

	/* insert new chunk and child */
	n48->slot_idxs[chunk] = insertpos;
	n48->children[insertpos] = child;
	n48->base.count++;

	/* Update fast path */
	tree->ctl->fp = node.alloc;
	tree->ctl->fp_ref = parent_slot;
	tree->ctl->fp_leaf = child;
	tree->ctl->fp_shift = 0;  /* Parent of leaves is at shift 0 */

	RT_VERIFY_NODE((RT_NODE *) n48);
}

/* Add child to Node256 and update fast path */
static inline void
RT_QUART_ADD_CHILD_256_CHANGE_FP(RT_RADIX_TREE *tree, RT_PTR_ALLOC *parent_slot,
								 RT_CHILD_PTR node, uint8 chunk, RT_PTR_ALLOC child)
{
	RT_NODE_256 *n256 = (RT_NODE_256 *) node.local;
	int idx = RT_BM_IDX(chunk);
	int bitnum = RT_BM_BIT(chunk);

	/* mark the slot used */
	n256->isset[idx] |= ((bitmapword) 1 << bitnum);

	/* insert new child */
	n256->children[chunk] = child;
	n256->base.count++;

	/* Update fast path */
	tree->ctl->fp = node.alloc;
	tree->ctl->fp_ref = parent_slot;
	tree->ctl->fp_leaf = child;
	tree->ctl->fp_shift = 0;  /* Parent of leaves is at shift 0 */

	RT_VERIFY_NODE((RT_NODE *) n256);
}

/*
 * Node-specific insertion functions that preserve fast path (preserve_fp variants)
 */

/* Add child to Node4 without changing fast path */
static inline void
RT_QUART_ADD_CHILD_4_PRESERVE_FP(RT_RADIX_TREE *tree, RT_CHILD_PTR node,
								 uint8 chunk, RT_PTR_ALLOC child)
{
	RT_NODE_4 *n4 = (RT_NODE_4 *) node.local;
	int count = n4->base.count;
	int insertpos = RT_NODE_4_GET_INSERTPOS(n4, chunk, count);

	/* shift chunks and children */
	RT_SHIFT_ARRAYS_FOR_INSERT(n4->chunks, n4->children, count, insertpos);

	/* insert new chunk and child */
	n4->chunks[insertpos] = chunk;
	n4->children[insertpos] = child;
	n4->base.count++;

	RT_VERIFY_NODE((RT_NODE *) n4);
}

/* Add child to Node16 without changing fast path */
static inline void
RT_QUART_ADD_CHILD_16_PRESERVE_FP(RT_RADIX_TREE *tree, RT_CHILD_PTR node,
								  uint8 chunk, RT_PTR_ALLOC child)
{
	RT_NODE_16 *n16 = (RT_NODE_16 *) node.local;
	int insertpos = RT_NODE_16_GET_INSERTPOS(n16, chunk);

	/* shift chunks and children */
	RT_SHIFT_ARRAYS_FOR_INSERT(n16->chunks, n16->children, n16->base.count, insertpos);

	/* insert new chunk and child */
	n16->chunks[insertpos] = chunk;
	n16->children[insertpos] = child;
	n16->base.count++;

	RT_VERIFY_NODE((RT_NODE *) n16);
}

/* Add child to Node48 without changing fast path */
static inline void
RT_QUART_ADD_CHILD_48_PRESERVE_FP(RT_RADIX_TREE *tree, RT_CHILD_PTR node,
								  uint8 chunk, RT_PTR_ALLOC child)
{
	RT_NODE_48 *n48 = (RT_NODE_48 *) node.local;
	int insertpos = n48->base.count;
	int idx = RT_BM_IDX(insertpos);
	int bitnum = RT_BM_BIT(insertpos);

	/* mark the slot used */
	n48->isset[idx] |= ((bitmapword) 1 << bitnum);

	/* insert new chunk and child */
	n48->slot_idxs[chunk] = insertpos;
	n48->children[insertpos] = child;
	n48->base.count++;

	RT_VERIFY_NODE((RT_NODE *) n48);
}

/* Add child to Node256 without changing fast path */
static inline void
RT_QUART_ADD_CHILD_256_PRESERVE_FP(RT_RADIX_TREE *tree, RT_CHILD_PTR node,
								   uint8 chunk, RT_PTR_ALLOC child)
{
	RT_NODE_256 *n256 = (RT_NODE_256 *) node.local;
	int idx = RT_BM_IDX(chunk);
	int bitnum = RT_BM_BIT(chunk);

	/* mark the slot used */
	n256->isset[idx] |= ((bitmapword) 1 << bitnum);

	/* insert new child */
	n256->children[chunk] = child;
	n256->base.count++;

	RT_VERIFY_NODE((RT_NODE *) n256);
}

/*
 * QuART-specific helper functions for node insertion with fast path management
 */

/*
 * RT_NODE_INSERT variant that updates fast path (change_fp)
 * Returns pointer to newly inserted child slot.
 */
static pg_noinline RT_PTR_ALLOC *
RT_NODE_INSERT_CHANGE_FP(RT_RADIX_TREE *tree, RT_PTR_ALLOC *parent_slot,
						 RT_CHILD_PTR node, uint8 chunk, RT_PTR_ALLOC child)
{
	RT_NODE *n = node.local;

	switch (n->kind)
	{
		case RT_NODE_KIND_4:
			{
				if (unlikely(RT_NODE_MUST_GROW(n)))
				{
					RT_PTR_ALLOC *slot = RT_GROW_NODE_4(tree, parent_slot, node, chunk);
					*slot = child;
					/* After grow, need to update fast path with new node */
					RT_CHILD_PTR newnode;
					newnode.alloc = *parent_slot;
					RT_PTR_SET_LOCAL(tree, &newnode);
					tree->ctl->fp = newnode.alloc;
					tree->ctl->fp_ref = parent_slot;
					tree->ctl->fp_leaf = child;
					return slot;
				}
				RT_NODE_4 *n4 = (RT_NODE_4 *) node.local;
				int pos = RT_NODE_4_GET_INSERTPOS(n4, chunk, n4->base.count);
				RT_QUART_ADD_CHILD_4_CHANGE_FP(tree, parent_slot, node, chunk, child);
				return &n4->children[pos];
			}
		case RT_NODE_KIND_16:
			{
				if (unlikely(RT_NODE_MUST_GROW(n)))
				{
					RT_PTR_ALLOC *slot = RT_GROW_NODE_16(tree, parent_slot, node, chunk);
					*slot = child;
					RT_CHILD_PTR newnode;
					newnode.alloc = *parent_slot;
					RT_PTR_SET_LOCAL(tree, &newnode);
					tree->ctl->fp = newnode.alloc;
					tree->ctl->fp_ref = parent_slot;
					tree->ctl->fp_leaf = child;
					tree->ctl->fp_shift = 0;  /* Parent of leaves is at shift 0 */
					return slot;
				}
				RT_NODE_16 *n16 = (RT_NODE_16 *) node.local;
				int pos = RT_NODE_16_GET_INSERTPOS(n16, chunk);
				RT_QUART_ADD_CHILD_16_CHANGE_FP(tree, parent_slot, node, chunk, child);
				return &n16->children[pos];
			}
		case RT_NODE_KIND_48:
			{
				if (unlikely(RT_NODE_MUST_GROW(n)))
				{
					RT_PTR_ALLOC *slot = RT_GROW_NODE_48(tree, parent_slot, node, chunk);
					*slot = child;
					RT_CHILD_PTR newnode;
					newnode.alloc = *parent_slot;
					RT_PTR_SET_LOCAL(tree, &newnode);
					tree->ctl->fp = newnode.alloc;
					tree->ctl->fp_ref = parent_slot;
					tree->ctl->fp_leaf = child;
					tree->ctl->fp_shift = 0;  /* Parent of leaves is at shift 0 */
					return slot;
				}
				RT_NODE_48 *n48 = (RT_NODE_48 *) node.local;
				int insertpos = n48->base.count;
				RT_QUART_ADD_CHILD_48_CHANGE_FP(tree, parent_slot, node, chunk, child);
				return &n48->children[insertpos];
			}
		case RT_NODE_KIND_256:
			{
				RT_QUART_ADD_CHILD_256_CHANGE_FP(tree, parent_slot, node, chunk, child);
				return RT_NODE_256_GET_CHILD((RT_NODE_256 *) node.local, chunk);
			}
		default:
			pg_unreachable();
	}
}

/*
 * RT_NODE_INSERT variant that preserves fast path (preserve_fp)
 * Returns pointer to newly inserted child slot.
 */
static pg_noinline RT_PTR_ALLOC *
RT_NODE_INSERT_PRESERVE_FP(RT_RADIX_TREE *tree, RT_PTR_ALLOC *parent_slot,
						   RT_CHILD_PTR node, uint8 chunk, RT_PTR_ALLOC child)
{
	RT_NODE *n = node.local;

	switch (n->kind)
	{
		case RT_NODE_KIND_4:
			{
				if (unlikely(RT_NODE_MUST_GROW(n)))
				{
					RT_PTR_ALLOC *slot = RT_GROW_NODE_4(tree, parent_slot, node, chunk);
					*slot = child;
					return slot;
				}
				RT_NODE_4 *n4 = (RT_NODE_4 *) node.local;
				int pos = RT_NODE_4_GET_INSERTPOS(n4, chunk, n4->base.count);
				RT_QUART_ADD_CHILD_4_PRESERVE_FP(tree, node, chunk, child);
				return &n4->children[pos];
			}
		case RT_NODE_KIND_16:
			{
				if (unlikely(RT_NODE_MUST_GROW(n)))
				{
					RT_PTR_ALLOC *slot = RT_GROW_NODE_16(tree, parent_slot, node, chunk);
					*slot = child;
					return slot;
				}
				RT_NODE_16 *n16 = (RT_NODE_16 *) node.local;
				int pos = RT_NODE_16_GET_INSERTPOS(n16, chunk);
				RT_QUART_ADD_CHILD_16_PRESERVE_FP(tree, node, chunk, child);
				return &n16->children[pos];
			}
		case RT_NODE_KIND_48:
			{
				if (unlikely(RT_NODE_MUST_GROW(n)))
				{
					RT_PTR_ALLOC *slot = RT_GROW_NODE_48(tree, parent_slot, node, chunk);
					*slot = child;
					return slot;
				}
				RT_NODE_48 *n48 = (RT_NODE_48 *) node.local;
				int insertpos = n48->base.count;
				RT_QUART_ADD_CHILD_48_PRESERVE_FP(tree, node, chunk, child);
				return &n48->children[insertpos];
			}
		case RT_NODE_KIND_256:
			{
				RT_QUART_ADD_CHILD_256_PRESERVE_FP(tree, node, chunk, child);
				return RT_NODE_256_GET_CHILD((RT_NODE_256 *) node.local, chunk);
			}
		default:
			pg_unreachable();
	}
}

/*
 * Recursive insert that updates fast path (change_fp)
 * 
 * Implements the QuART stail_reset_bidir algorithm for sequential insertions.
 * This version updates the fast path as it traverses, enabling optimized
 * sequential access patterns.
 */
static RT_PTR_ALLOC *
RT_QUART_INSERT_RECURSIVE_CHANGE_FP(RT_RADIX_TREE *tree, RT_PTR_ALLOC *parent_slot,
									uint64 key, int shift, bool *found)
{
	RT_PTR_ALLOC *slot;
	RT_CHILD_PTR node;
	uint8 chunk = RT_GET_KEY_CHUNK(key, shift);

	node.alloc = *parent_slot;
	RT_PTR_SET_LOCAL(tree, &node);
	slot = RT_NODE_SEARCH(node.local, chunk);

	if (slot == NULL)
	{
		*found = false;

		if (shift == 0)
		{
			/* Leaf level - insert with fast path update */
			RT_PTR_ALLOC child = 0;  /* Will be set by caller */
			slot = RT_NODE_INSERT_CHANGE_FP(tree, parent_slot, node, chunk, child);
			return slot;
		}
		else
		{
			/* Inner node - allocate child node and recurse */
			RT_CHILD_PTR child;
			RT_NODE_4 *n4;

			child = RT_ALLOC_NODE(tree, RT_NODE_KIND_4, RT_CLASS_4);
			n4 = (RT_NODE_4 *) child.local;
			n4->base.count = 0;

			/* Insert child WITHOUT updating fast path (only update at leaf level) */
			slot = RT_NODE_INSERT_PRESERVE_FP(tree, parent_slot, node, chunk, child.alloc);

			/* Track in fast path */
			tree->ctl->fp_path[tree->ctl->fp_path_length] = child.alloc;
			tree->ctl->fp_path_length++;

			/* Recurse down */
			return RT_QUART_INSERT_RECURSIVE_CHANGE_FP(tree, slot, key, shift - RT_SPAN, found);
		}
	}
	else
	{
		if (shift == 0)
		{
			*found = true;
			return slot;
		}
		else
		{
			/* Track this node in fast path */
			tree->ctl->fp_path[tree->ctl->fp_path_length] = *slot;
			tree->ctl->fp_path_length++;

			return RT_QUART_INSERT_RECURSIVE_CHANGE_FP(tree, slot, key, shift - RT_SPAN, found);
		}
	}
}

/*
 * Recursive insert that preserves fast path (preserve_fp)
 * 
 * Implements non-sequential insertion for QuART stail_reset_bidir.
 * This version does NOT update the fast path, preserving the existing
 * state for future sequential accesses.
 */
static RT_PTR_ALLOC *
RT_QUART_INSERT_RECURSIVE_PRESERVE_FP(RT_RADIX_TREE *tree, RT_PTR_ALLOC *parent_slot,
									  uint64 key, int shift, bool *found)
{
	RT_PTR_ALLOC *slot;
	RT_CHILD_PTR node;
	uint8 chunk = RT_GET_KEY_CHUNK(key, shift);

	node.alloc = *parent_slot;
	RT_PTR_SET_LOCAL(tree, &node);
	slot = RT_NODE_SEARCH(node.local, chunk);

	if (slot == NULL)
	{
		*found = false;

		if (shift == 0)
		{
			/* Leaf level - insert without fast path update */
			RT_PTR_ALLOC child = 0;  /* Will be set by caller */
			slot = RT_NODE_INSERT_PRESERVE_FP(tree, parent_slot, node, chunk, child);
			return slot;
		}
		else
		{
			/* Inner node - allocate child node and recurse */
			RT_CHILD_PTR child;
			RT_NODE_4 *n4;

			child = RT_ALLOC_NODE(tree, RT_NODE_KIND_4, RT_CLASS_4);
			n4 = (RT_NODE_4 *) child.local;
			n4->base.count = 0;

			/* Insert child without updating fast path */
			slot = RT_NODE_INSERT_PRESERVE_FP(tree, parent_slot, node, chunk, child.alloc);

			/* Recurse down */
			return RT_QUART_INSERT_RECURSIVE_PRESERVE_FP(tree, slot, key, shift - RT_SPAN, found);
		}
	}
	else
	{
		if (shift == 0)
		{
			*found = true;
			return slot;
		}
		else
			return RT_QUART_INSERT_RECURSIVE_PRESERVE_FP(tree, slot, key, shift - RT_SPAN, found);
	}
}

/*
 * QuART stail_reset_bidir main insertion function
 * 
 * Implements the exact algorithm from QuART_stail_reset_bidir.h:
 * - Byte-by-byte comparison of first 3 bytes against fp_leaf_value
 * - Bidirectional pattern detection (ascending/descending)
 * - Bridge value handling for byte boundary crossings
 * - Reset counter for non-sequential inserts
 * - Fast path insert when first 3 bytes match fp_leaf_value
 */
RT_SCOPE bool
RT_SET_QUART(RT_RADIX_TREE *tree, uint64 key, RT_VALUE_TYPE *value_p)
{
	bool found;
	RT_PTR_ALLOC *slot;
	size_t value_sz = RT_GET_VALUE_SIZE(value_p);
	uint64 fp_leaf_value;
	bool use_change_fp = false;
	bool use_preserve_fp = false;
	bool is_fp_insert = false;

#ifdef RT_SHMEM
	Assert(tree->ctl->magic == RT_RADIX_TREE_MAGIC);
#endif

	Assert(RT_PTR_ALLOC_IS_VALID(tree->ctl->root));

	/* First insert - always use change_fp to initialize fast path */
	if (tree->ctl->num_keys == 0)
	{
		tree->ctl->fp_dir = true;
		tree->ctl->fp_reset_counter = 300;
		tree->ctl->fp_initialized = true;
		tree->ctl->fp_path_length = 1;
		tree->ctl->fp_path[0] = tree->ctl->root;
		use_change_fp = true;
		goto do_insert;
	}

	/* Extend the tree if necessary */
	if (unlikely(key > tree->ctl->max_val))
		RT_EXTEND_UP(tree, key);

	/* Get fp_leaf value for comparison */
	fp_leaf_value = tree->ctl->fp_last_key;

	/*
	 * Compare first 3 bytes of key against fp_leaf_value.
	 * This implements the core QuART stail_reset_bidir algorithm.
	 */
	if (tree->ctl->fp_dir)
	{
		/* ASCENDING MODE */
		for (int i = 0; i < 3; i++)
		{
			uint8 key_byte = (key >> (8 * (3 - i))) & 0xFF;
			uint8 leaf_byte = (fp_leaf_value >> (8 * (3 - i))) & 0xFF;

			if (key_byte == leaf_byte)
				continue;  /* Byte matches, check next byte */

			if (key_byte < leaf_byte)
			{
				/* Key is less - non-sequential insert with preserve_fp */
				tree->ctl->fp_reset_counter--;
				use_preserve_fp = true;
				goto do_insert;
			}

			/* key_byte > leaf_byte - check for bridge pattern */
			if (i == 0)
			{
				/* First byte: check for X,255,255 -> X+1,0,0 */
				if ((key_byte == leaf_byte + 1) && ((key >> 8) & 0xFFFF) == 0 &&
					((fp_leaf_value >> 8) & 0xFFFF) == 0xFFFF)
				{
					/* Bridge value - use change_fp */
					tree->ctl->fp_reset_counter = 300;
					tree->ctl->fp_path_length = 1;
					tree->ctl->fp_path[0] = tree->ctl->root;
					use_change_fp = true;
					goto do_insert;
				}
			}
			else if (i == 1)
			{
				/* Second byte: check for X,Y,255 -> X,Y+1,0 */
				if ((key_byte == leaf_byte + 1) && ((key >> 8) & 0xFF) == 0 &&
					((key >> 24) == (fp_leaf_value >> 24)) &&
					((fp_leaf_value >> 8) & 0xFF) == 0xFF)
				{
					tree->ctl->fp_reset_counter = 300;
					tree->ctl->fp_path_length = 1;
					tree->ctl->fp_path[0] = tree->ctl->root;
					use_change_fp = true;
					goto do_insert;
				}
			}
			else  /* i == 2 */
			{
				/* Third byte: check for X,Y,Z -> X,Y,Z+1 */
				if ((key_byte == leaf_byte + 1) &&
					((key >> 16) == (fp_leaf_value >> 16)))
				{
					tree->ctl->fp_reset_counter = 300;
					tree->ctl->fp_path_length = 1;
					tree->ctl->fp_path[0] = tree->ctl->root;
					use_change_fp = true;
					goto do_insert;
				}
			}

			/* Not a bridge - non-sequential insert */
			if (tree->ctl->fp_reset_counter <= 0)
			{
				tree->ctl->fp_reset_counter = 300;
				tree->ctl->fp_path_length = 1;
				tree->ctl->fp_path[0] = tree->ctl->root;
				tree->ctl->fp_dir = true;
				use_change_fp = true;
			}
			else
			{
				tree->ctl->fp_reset_counter--;
				use_preserve_fp = true;
			}
			goto do_insert;
		}

		/* All 3 bytes match - check last byte for direction */
		uint8 key_last = key & 0xFF;
		uint8 leaf_last = fp_leaf_value & 0xFF;

		if (key_last < leaf_last)
		{
			/* Direction change to descending */
			tree->ctl->fp_dir = false;
			/* Update fp_last_key with new last byte */
			tree->ctl->fp_last_key = (fp_leaf_value & 0xFFFFFF00ULL) | key_last;
		}
		/* Fast path insert will happen below */
	}
	else
	{
		/* DESCENDING MODE */
		for (int i = 0; i < 3; i++)
		{
			uint8 key_byte = (key >> (8 * (3 - i))) & 0xFF;
			uint8 leaf_byte = (fp_leaf_value >> (8 * (3 - i))) & 0xFF;

			if (key_byte == leaf_byte)
				continue;

			if (key_byte > leaf_byte)
			{
				/* Key is greater - non-sequential insert */
				tree->ctl->fp_reset_counter--;
				use_preserve_fp = true;
				goto do_insert;
			}

			/* key_byte < leaf_byte - check for bridge pattern */
			if (i == 0)
			{
				/* First byte: check for X,0,0 -> X-1,255,255 */
				if ((leaf_byte == key_byte + 1) && ((fp_leaf_value >> 8) & 0xFFFF) == 0 &&
					((key >> 8) & 0xFFFF) == 0xFFFF)
				{
					tree->ctl->fp_reset_counter = 300;
					tree->ctl->fp_path_length = 1;
					tree->ctl->fp_path[0] = tree->ctl->root;
					use_change_fp = true;
					goto do_insert;
				}
			}
			else if (i == 1)
			{
				/* Second byte: check for X,Y,0 -> X,Y-1,255 */
				if ((leaf_byte == key_byte + 1) && ((fp_leaf_value >> 8) & 0xFF) == 0 &&
					((key >> 24) == (fp_leaf_value >> 24)) &&
					((key >> 8) & 0xFF) == 0xFF)
				{
					tree->ctl->fp_reset_counter = 300;
					tree->ctl->fp_path_length = 1;
					tree->ctl->fp_path[0] = tree->ctl->root;
					use_change_fp = true;
					goto do_insert;
				}
			}
			else  /* i == 2 */
			{
				/* Third byte: check for X,Y,Z -> X,Y,Z-1 */
				if ((leaf_byte == key_byte + 1) &&
					((key >> 16) == (fp_leaf_value >> 16)))
				{
					tree->ctl->fp_reset_counter = 300;
					tree->ctl->fp_path_length = 1;
					tree->ctl->fp_path[0] = tree->ctl->root;
					use_change_fp = true;
					goto do_insert;
				}
			}

			/* Not a bridge - non-sequential insert */
			if (tree->ctl->fp_reset_counter <= 0)
			{
				tree->ctl->fp_reset_counter = 300;
				tree->ctl->fp_path_length = 1;
				tree->ctl->fp_path[0] = tree->ctl->root;
				tree->ctl->fp_dir = true;
				use_change_fp = true;
			}
			else
			{
				tree->ctl->fp_reset_counter--;
				use_preserve_fp = true;
			}
			goto do_insert;
		}

		/* All 3 bytes match - check last byte for direction */
		uint8 key_last = key & 0xFF;
		uint8 leaf_last = fp_leaf_value & 0xFF;

		if (key_last > leaf_last)
		{
			/* Direction change to ascending */
			tree->ctl->fp_dir = true;
			/* Update fp_last_key with new last byte */
			tree->ctl->fp_last_key = (fp_leaf_value & 0xFFFFFF00ULL) | key_last;
		}
	}

	/*
	 * If we reach here, first 3 bytes matched fp_leaf_value.
	 * This is a fast path insert - insert using preserve_fp.
	 */
	is_fp_insert = true;
	tree->ctl->fp_reset_counter = 300;

do_insert:
	if (use_change_fp)
	{
		tree->ctl->fp_path_length = 1;
		tree->ctl->fp_path[0] = tree->ctl->root;
		slot = RT_QUART_INSERT_RECURSIVE_CHANGE_FP(tree, &tree->ctl->root,
												   key, tree->ctl->start_shift, &found);
	}
	else if (is_fp_insert)
	{
		/* Fast path insert - use cached fp node directly */
		RT_CHILD_PTR node;
		uint8 chunk;
		
		/* If fp is at shift 0, it's a leaf parent, use shift 0 for chunk extraction.
		 * Otherwise traverse down from fp at its shift level. */
		if (tree->ctl->fp_shift == 0)
		{
			/* Fast path is at leaf parent level */
			chunk = RT_GET_KEY_CHUNK(key, 0);
			node.alloc = tree->ctl->fp;
			RT_PTR_SET_LOCAL(tree, &node);
			slot = RT_NODE_SEARCH(node.local, chunk);
			
			if (slot == NULL)
			{
				/* Need to insert new leaf */
				RT_PTR_ALLOC child = 0;  /* Will be set below in have_slot */
				RT_PTR_ALLOC old_fp = node.alloc;
				slot = RT_NODE_INSERT_PRESERVE_FP(tree, tree->ctl->fp_ref, node, chunk, child);
				found = false;
				
				/* If the node grew during insert, update fp to point to the new node */
				if (*tree->ctl->fp_ref != old_fp)
				{
					tree->ctl->fp = *tree->ctl->fp_ref;
				}
			}
			else
			{
				/* Leaf already exists */
				found = true;
			}
		}
		else
		{
			/* Need to traverse down from fp - fallback to preserve_fp recursion */
			slot = RT_QUART_INSERT_RECURSIVE_PRESERVE_FP(tree, tree->ctl->fp_ref,
													 key, tree->ctl->fp_shift, &found);
		}
	}
	else if (use_preserve_fp)
	{
		slot = RT_QUART_INSERT_RECURSIVE_PRESERVE_FP(tree, &tree->ctl->root,
													 key, tree->ctl->start_shift, &found);
	}
	else
	{
		/* Shouldn't reach here */
		Assert(false);
		slot = NULL;
	}

have_slot:
	Assert(slot != NULL);

	if (RT_VALUE_IS_EMBEDDABLE(value_p))
	{
		if (found && !RT_CHILDPTR_IS_VALUE(*slot))
			RT_FREE_LEAF(tree, *slot);

		memcpy(slot, value_p, value_sz);

#ifdef RT_RUNTIME_EMBEDDABLE_VALUE
#ifdef RT_SHMEM
		*slot |= 1;
#else
		*((uintptr_t *) slot) |= 1;
#endif
#endif
	}
	else
	{
		RT_CHILD_PTR leaf;

		if (found && !RT_CHILDPTR_IS_VALUE(*slot))
		{
			Assert(RT_PTR_ALLOC_IS_VALID(*slot));
			leaf.alloc = *slot;
			RT_PTR_SET_LOCAL(tree, &leaf);

			if (RT_GET_VALUE_SIZE((RT_VALUE_TYPE *) leaf.local) != value_sz)
			{
				RT_FREE_LEAF(tree, *slot);
				leaf = RT_ALLOC_LEAF(tree, value_sz);
				*slot = leaf.alloc;
			}
		}
		else
		{
			leaf = RT_ALLOC_LEAF(tree, value_sz);
			*slot = leaf.alloc;
		}

		memcpy(leaf.local, value_p, value_sz);
	}

	/* Update fast path state */
	tree->ctl->fp_last_key = key;
	if (use_change_fp && !found)
		tree->ctl->fp_leaf = *slot;

	if (!found)
		tree->ctl->num_keys++;

	return found;
}