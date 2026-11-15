//! Groot's tree layout, `RecursiveNodeReorder` (Groot `bt_editor/utils.cpp`), unchanged.
//!
//! Each level keeps a cursor; children start centred under their parent but never left of
//! (above, horizontally) what is already on their level, and a parent is then re-centred over
//! its first and last child. Finally each level is moved to sit below the tallest node of the
//! level before it.

use crate::model::{Tree, Uid};

pub const LEVEL_SPACING: f32 = 80.0;
pub const NODE_SPACING: f32 = 40.0;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vec2 {
    pub x: f32,
    pub y: f32,
}

impl Vec2 {
    pub const fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }
}

/// Root on top, children below (Groot's default) or root left, children to the right.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Orientation {
    #[default]
    Vertical,
    Horizontal,
}

/// Top-left corner of every node; `None` for nodes inside a collapsed SubTree.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Layout {
    pub positions: Vec<Option<Vec2>>,
}

/// Lay out `tree` given each node's size.
///
/// `expanded(uid)` decides whether a node's children are shown (Groot collapses SubTrees).
pub fn layout(
    tree: &Tree,
    sizes: &[Vec2],
    orientation: Orientation,
    expanded: impl Fn(Uid) -> bool,
) -> Layout {
    let mut state = State {
        tree,
        sizes,
        vertical: orientation == Orientation::Vertical,
        expanded: &expanded,
        pos: vec![None; tree.len()],
        layer_cursor: Vec::new(),
        nodes_by_level: vec![Vec::new()],
    };
    if tree.is_empty() {
        return Layout::default();
    }
    let root = sizes[0];
    state
        .layer_cursor
        .push(Vec2::new(-root.x * 0.5, -root.y * 0.5));
    state.step(0, 0);

    // Stack the levels.
    let vertical = state.vertical;
    let mut offset = if vertical { root.y } else { root.x } + LEVEL_SPACING;
    for level in state.nodes_by_level.iter().skip(1) {
        let mut max_extent: f32 = 0.0;
        for &uid in level {
            let size = sizes[uid];
            let p = state.pos[uid].as_mut().expect("placed");
            if vertical {
                max_extent = max_extent.max(size.y);
                p.y = offset;
            } else {
                max_extent = max_extent.max(size.x);
                p.x = offset;
            }
        }
        offset += max_extent + LEVEL_SPACING;
    }
    Layout {
        positions: state.pos,
    }
}

struct State<'a, F: Fn(Uid) -> bool> {
    tree: &'a Tree,
    sizes: &'a [Vec2],
    vertical: bool,
    expanded: &'a F,
    pos: Vec<Option<Vec2>>,
    layer_cursor: Vec<Vec2>,
    nodes_by_level: Vec<Vec<Uid>>,
}

impl<F: Fn(Uid) -> bool> State<'_, F> {
    /// Along the level (x when vertical).
    fn along(&self, v: Vec2) -> f32 {
        if self.vertical { v.x } else { v.y }
    }

    fn along_mut<'v>(&self, v: &'v mut Vec2) -> &'v mut f32 {
        if self.vertical { &mut v.x } else { &mut v.y }
    }

    fn step(&mut self, level: usize, uid: Uid) {
        let node_pos = self.layer_cursor[level];
        self.pos[uid] = Some(node_pos);
        self.nodes_by_level[level].push(uid);

        let children: &[Uid] = if (self.expanded)(uid) {
            &self.tree.nodes[uid].children
        } else {
            &[]
        };
        if children.is_empty() {
            return;
        }
        let child_level = level + 1;
        let size = self.sizes[uid];

        let mut recommended = NODE_SPACING * 0.5 + self.along(node_pos) + self.along(size) * 0.5;
        for &child in children {
            recommended -= (self.along(self.sizes[child]) + NODE_SPACING) * 0.5;
        }
        if child_level >= self.layer_cursor.len() {
            let cursor = if self.vertical {
                Vec2::new(recommended, node_pos.y + LEVEL_SPACING)
            } else {
                Vec2::new(node_pos.x + LEVEL_SPACING, recommended)
            };
            self.layer_cursor.push(cursor);
            self.nodes_by_level.push(Vec::new());
        } else {
            let mut cursor = self.layer_cursor[child_level];
            let current = self.along(cursor);
            *self.along_mut(&mut cursor) = recommended.max(current);
            self.layer_cursor[child_level] = cursor;
        }

        for &child in children {
            self.step(child_level, child);
            let advance = self.along(self.sizes[child]) + NODE_SPACING;
            let mut cursor = self.layer_cursor[child_level];
            *self.along_mut(&mut cursor) += advance;
            self.layer_cursor[child_level] = cursor;
        }

        // Re-balance the parent over its first and last child.
        let centre =
            |s: &Self, c: Uid| s.along(s.pos[c].expect("placed")) + s.along(s.sizes[c]) * 0.5;
        let first = centre(self, children[0]);
        let last = centre(self, *children.last().expect("non-empty"));
        let new_along = (first + last) * 0.5 - self.along(size) * 0.5;
        let shift = new_along - self.along(node_pos);

        let mut p = self.pos[uid].expect("placed");
        *self.along_mut(&mut p) += shift;
        self.pos[uid] = Some(p);
        let mut cursor = self.layer_cursor[level];
        *self.along_mut(&mut cursor) += shift;
        self.layer_cursor[level] = cursor;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::Document;

    fn sample() -> Tree {
        let path = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../test_data/crossdoor_with_subtree.xml"
        );
        let xml = std::fs::read_to_string(path).unwrap();
        Document::parse(&xml)
            .unwrap()
            .instantiate("MainTree")
            .unwrap()
    }

    fn overlaps(a: (Vec2, Vec2), b: (Vec2, Vec2)) -> bool {
        a.0.x < b.0.x + b.1.x
            && b.0.x < a.0.x + a.1.x
            && a.0.y < b.0.y + b.1.y
            && b.0.y < a.0.y + a.1.y
    }

    fn check(orientation: Orientation) {
        let tree = sample();
        let sizes: Vec<Vec2> = (0..tree.len())
            .map(|i| Vec2::new(100.0 + 13.0 * (i % 4) as f32, 50.0 + 7.0 * (i % 3) as f32))
            .collect();
        let layout = layout(&tree, &sizes, orientation, |_| true);
        let placed: Vec<(Vec2, Vec2)> = layout
            .positions
            .iter()
            .zip(&sizes)
            .map(|(p, s)| (p.unwrap(), *s))
            .collect();
        for i in 0..placed.len() {
            for j in i + 1..placed.len() {
                assert!(!overlaps(placed[i], placed[j]), "{i} overlaps {j}");
            }
        }
        // Parents are centred over their first and last child.
        for node in &tree.nodes {
            if node.children.is_empty() {
                continue;
            }
            let c = |uid: Uid| match orientation {
                Orientation::Vertical => placed[uid].0.x + placed[uid].1.x * 0.5,
                Orientation::Horizontal => placed[uid].0.y + placed[uid].1.y * 0.5,
            };
            let expected = (c(node.children[0]) + c(*node.children.last().unwrap())) * 0.5;
            assert!((c(node.uid) - expected).abs() < 1e-3, "node {}", node.uid);
        }
        // Children are past their parent along the depth axis.
        for node in tree.nodes.iter().skip(1) {
            let parent = node.parent.unwrap();
            match orientation {
                Orientation::Vertical => {
                    assert!(placed[node.uid].0.y > placed[parent].0.y + placed[parent].1.y)
                }
                Orientation::Horizontal => {
                    assert!(placed[node.uid].0.x > placed[parent].0.x + placed[parent].1.x)
                }
            }
        }
    }

    #[test]
    fn vertical_has_no_overlaps_and_centred_parents() {
        check(Orientation::Vertical);
    }

    #[test]
    fn horizontal_has_no_overlaps_and_centred_parents() {
        check(Orientation::Horizontal);
    }

    #[test]
    fn collapsed_subtrees_hide_their_nodes() {
        let tree = sample();
        let sizes = vec![Vec2::new(80.0, 40.0); tree.len()];
        let layout = layout(&tree, &sizes, Orientation::Vertical, |uid| uid != 5);
        assert!(layout.positions[5].is_some());
        assert!(layout.positions[6..=12].iter().all(Option::is_none));
        assert!(layout.positions[13].is_some());
    }
}
