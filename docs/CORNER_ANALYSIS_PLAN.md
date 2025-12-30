# Corner Analysis Implementation Plan for Brink Fixing

## Overview

The current brink fixing system only analyzes **edges** (1D objects where two faces meet).
The TODO at `brink.cpp:8` states: "we should consider corners in addition to brinks."

Corner stickiness occurs when multiple edges meet at a single vertex and there's a
collision mismatch that isn't detected by edge-only analysis.

## Current System Architecture

### Data Structures (brink.cpp:220-309)

```
btreepoint_t (0D) - Corners/Vertices
    ├── vec3_t v           - position
    ├── bool infinite      - on bounding box?
    └── btreeedge_l *edges - all edges meeting at this point

btreeedge_t (1D) - Edges (CURRENTLY ANALYZED)
    ├── btreepoint_r points[2] - start/end points
    ├── btreeface_l *faces     - faces sharing this edge
    └── bbrink_t *brink        - brink analysis data

btreeface_t (2D) - Faces
    ├── btreeedge_l *edges     - edges forming this face
    └── btreeleaf_r leafs[2]   - leaves on each side

btreeleaf_t (3D) - BSP Leaves
    ├── btreeface_l *faces     - faces bounding this leaf
    └── bclipnode_t *clipnode  - collision data
```

### Current Flow (brink.cpp:1810-1829)

```
CreateBrinkinfo()
    ├── ExpandClipnodes()   - Build clipnode tree
    ├── BuildTreeCells()    - Build geometric tree (points, edges, faces, leaves)
    ├── CollectBrinks()     - Gather brinks from EDGES only
    ├── AnalyzeBrinks()     - Analyze each brink using bcircle_t (2D circle around edge)
    ├── FreeBrinks()
    ├── DeleteTreeCells()
    └── SortPartitions()    - Organize fixes by priority
```

### Brink Analysis (brink.cpp:1502-1700)

For each edge:
1. Build `bcircle_t` - a 2D representation of all possible movement directions around the edge
2. Find transitions between SOLID and EMPTY content
3. Classify as: BrinkFloorBlocking > BrinkFloor > BrinkWallBlocking > BrinkWall > BrinkAny
4. Add partition planes to fix collision issues

## Proposed Corner Analysis

### New Data Structure

```cpp
typedef struct bcorner_s
{
    vec3_t position;           // Corner location
    int numnodes;              // BSP nodes around corner
    std::vector<bbrinknode_t> *nodes;
    btreepoint_t *point;       // Reference to tree point
} bcorner_t;

// 3D sphere of analysis (vs 2D circle for edges)
typedef struct bsphere_s
{
    vec3_t center;
    // Spherical wedges representing content regions
    std::vector<bspherewedge_t> wedges;
} bsphere_t;
```

### Implementation Steps

#### Step 1: Add Corner Collection (after CollectBrinks)

```cpp
// New function in brink.cpp
void CollectCorners_r(bclipnode_t *node, int &numcorners, bcorner_t **corners)
{
    if (node->isleaf)
    {
        btreeface_l::iterator fi;
        btreeedge_l::iterator ei;
        for (fi = node->treeleaf->faces->begin(); fi != node->treeleaf->faces->end(); fi++)
        {
            for (ei = fi->f->edges->begin(); ei != fi->f->edges->end(); ei++)
            {
                // For each edge endpoint (corner)
                for (int side = 0; side < 2; side++)
                {
                    btreepoint_t *tp = GetPointFromEdge(ei->e, side);
                    if (tp->tmp_tested || tp->infinite)
                        continue;
                    tp->tmp_tested = true;

                    // Only analyze corners with 3+ edges (complex junctions)
                    if (tp->edges->size() >= 3)
                    {
                        if (corners != NULL)
                        {
                            corners[numcorners] = CreateCorner(tp);
                        }
                        numcorners++;
                    }
                }
            }
        }
    }
    else
    {
        CollectCorners_r(node->children[0], numcorners, corners);
        CollectCorners_r(node->children[1], numcorners, corners);
    }
}
```

#### Step 2: Create Corner Analysis

```cpp
bcorner_t *CreateCorner(btreepoint_t *tp)
{
    bcorner_t *c = (bcorner_t *)malloc(sizeof(bcorner_t));
    VectorCopy(tp->v, c->position);
    c->point = tp;
    c->numnodes = 1;
    c->nodes = new std::vector<bbrinknode_t>();

    // Initialize with leaf node
    bbrinknode_t newnode;
    newnode.isleaf = true;
    newnode.clipnode = NULL;
    c->nodes->push_back(newnode);

    return c;
}
```

#### Step 3: Analyze Corner Transitions

```cpp
void AnalyzeCorners(bbrinkinfo_t *info)
{
    for (int i = 0; i < info->numcorners; i++)
    {
        bcorner_t *c = info->corners[i];
        btreepoint_t *tp = c->point;

        // Collect all leaves adjacent to this corner
        std::set<bclipnode_t*> adjacentLeaves;
        for (btreeedge_l::iterator ei = tp->edges->begin();
             ei != tp->edges->end(); ei++)
        {
            for (btreeface_l::iterator fi = ei->e->faces->begin();
                 fi != ei->e->faces->end(); fi++)
            {
                for (int side = 0; side < 2; side++)
                {
                    btreeleaf_t *leaf = GetLeafFromFace(fi->f, side);
                    if (!leaf->infinite)
                    {
                        adjacentLeaves.insert(leaf->clipnode);
                    }
                }
            }
        }

        // Check for SOLID/EMPTY transitions
        bool hasSolid = false;
        bool hasEmpty = false;
        for (auto leaf : adjacentLeaves)
        {
            if (leaf->content == CONTENTS_SOLID)
                hasSolid = true;
            else
                hasEmpty = true;
        }

        // If both SOLID and EMPTY exist, analyze for stickiness
        if (hasSolid && hasEmpty)
        {
            AnalyzeCornerTransitions(c, adjacentLeaves);
        }
    }
}
```

#### Step 4: Modify CreateBrinkinfo Flow

```cpp
void *CreateBrinkinfo(const dclipnode_t *clipnodes, int headnode)
{
    bbrinkinfo_t *info;
    try
    {
        hlassume(info = (bbrinkinfo_t *)malloc(sizeof(bbrinkinfo_t)), assume_NoMemory);
        ExpandClipnodes(info, clipnodes, headnode);
        BuildTreeCells(info);

        // Existing edge analysis
        CollectBrinks(info);
        AnalyzeBrinks(info);
        FreeBrinks(info);

        // NEW: Corner analysis
        CollectCorners(info);
        AnalyzeCorners(info);
        FreeCorners(info);

        DeleteTreeCells(info);
        SortPartitions(info);
    }
    catch (std::bad_alloc)
    {
        hlassume(false, assume_NoMemory);
    }
    return info;
}
```

### Files to Modify

1. **brink.cpp** - Main implementation
   - Add bcorner_t structure
   - Add CollectCorners() and CollectCorners_r()
   - Add AnalyzeCorners()
   - Add corner classification (CornerFloorBlocking, etc.)
   - Modify CreateBrinkinfo() to call corner analysis

2. **bsp5.h** - Add declarations
   - Declare new corner-related structures and enums
   - Add numcorners and corners to bbrinkinfo_t

### Complexity Considerations

1. **3D vs 2D Analysis**: Edges use a 2D circle (bcircle_t), corners need 3D sphere analysis
2. **Multiple Edge Convergence**: Corners where 3+ edges meet are geometrically complex
3. **Performance**: More corners than edges to analyze, but most can be quickly rejected
4. **Priority System**: Need to integrate with existing BrinkFloorBlocking priority system

### Testing Strategy

1. Create test map with known corner stickiness issues
2. Compare clipnode output with/without corner analysis
3. Test in-game player movement around previously sticky corners
4. Verify no regression in edge-based brink fixing

### Estimated Effort

- **Structure definitions**: ~50 lines
- **Collection functions**: ~100 lines
- **Analysis functions**: ~300 lines
- **Integration**: ~50 lines
- **Testing/debugging**: Variable

Total: ~500 lines of new code

### Command-Line Flag

Add `-nocornerfix` flag to disable corner analysis (similar to `-nobrink`):

```cpp
// In bsp5.h
extern bool g_nocornerfix;

// In qbsp.cpp argument parsing
else if (!strcasecmp(argv[i], "-nocornerfix"))
{
    g_nocornerfix = true;
}
```
