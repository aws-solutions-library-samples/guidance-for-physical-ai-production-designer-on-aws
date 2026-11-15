//! Fixed-size bitset over op ids.

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct BitSet {
    words: Vec<u64>,
}

impl BitSet {
    pub fn new(n: usize) -> Self {
        Self {
            words: vec![0; n.div_ceil(64)],
        }
    }

    pub fn insert(&mut self, i: usize) {
        self.words[i / 64] |= 1 << (i % 64);
    }

    pub fn contains(&self, i: usize) -> bool {
        self.words
            .get(i / 64)
            .is_some_and(|w| w & (1 << (i % 64)) != 0)
    }

    pub fn union_with(&mut self, other: &BitSet) {
        for (a, b) in self.words.iter_mut().zip(&other.words) {
            *a |= b;
        }
    }

    pub fn iter(&self) -> impl Iterator<Item = usize> + '_ {
        self.words.iter().enumerate().flat_map(|(wi, &w)| {
            (0..64)
                .filter(move |b| w & (1 << b) != 0)
                .map(move |b| wi * 64 + b)
        })
    }

    pub fn len(&self) -> usize {
        self.words.iter().map(|w| w.count_ones() as usize).sum()
    }

    pub fn is_empty(&self) -> bool {
        self.words.iter().all(|&w| w == 0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn basic() {
        let mut a = BitSet::new(130);
        a.insert(0);
        a.insert(129);
        let mut b = BitSet::new(130);
        b.insert(64);
        a.union_with(&b);
        assert!(a.contains(64) && a.contains(129) && !a.contains(1));
        assert_eq!(a.iter().collect::<Vec<_>>(), vec![0, 64, 129]);
        assert_eq!(a.len(), 3);
    }
}
