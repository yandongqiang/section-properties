//! Pure text-matching helpers for the command palette (Phase 159-A).
//!
//! The palette ranks [`Command`](super::ui::commands::Command) entries by how
//! well their label / category / id match the typed query.  The matcher lives
//! here — in the GUI-free library — so its behaviour is unit-tested without a
//! display (see the `tests` module below).

/// Characters that start a new "word" for boundary scoring.
const BOUNDARY: &[char] = &[' ', '/', '-', '_', '(', ')', ':', '.', '·'];

/// Score how well `query` fuzzy-matches `text`, or `None` when `query` is not
/// a case-insensitive subsequence of `text`.
///
/// * An empty query matches everything with score `0`.
/// * `+1` per matched character.
/// * `+2` when a match continues the previous one (consecutive run).
/// * `+2` when a match starts at a word boundary (see [`BOUNDARY`]), and an
///   extra `+3` when it starts at the very beginning of the text.
///
/// Higher is better.  The score is deterministic: identical inputs always
/// produce the same result.
pub fn fuzzy_score(query: &str, text: &str) -> Option<i32> {
    let q: Vec<char> = query.to_lowercase().chars().collect();
    if q.is_empty() {
        return Some(0);
    }
    let t: Vec<char> = text.to_lowercase().chars().collect();
    let mut score = 0i32;
    let mut start = 0usize;
    let mut prev: Option<usize> = None;
    for &qc in &q {
        let mut found = None;
        for (i, &tc) in t.iter().enumerate().skip(start) {
            if tc == qc {
                found = Some(i);
                break;
            }
        }
        let i = found?;
        if prev == Some(i.wrapping_sub(1)) {
            score += 2; // consecutive run
        }
        if i == 0 {
            score += 5; // prefix of the whole text
        } else if BOUNDARY.contains(&t[i - 1]) {
            score += 2; // word boundary
        }
        score += 1;
        prev = Some(i);
        start = i + 1;
    }
    Some(score)
}

/// Best score of `query` against any of `haystacks` (first field wins ties),
/// or `None` when none match.
pub fn best_score(query: &str, haystacks: &[&str]) -> Option<i32> {
    haystacks.iter().filter_map(|h| fuzzy_score(query, h)).max()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_query_matches_everything() {
        assert_eq!(fuzzy_score("", "anything"), Some(0));
        assert_eq!(fuzzy_score("", ""), Some(0));
    }

    #[test]
    fn subsequence_is_required() {
        assert!(fuzzy_score("xyz", "Undo").is_none());
        assert!(fuzzy_score("un", "Undo").is_some());
        // Order matters: "od" is not a subsequence of "Undo".
        assert!(fuzzy_score("od", "Undo").is_none());
    }

    #[test]
    fn matching_is_case_insensitive() {
        assert_eq!(fuzzy_score("UNDO", "Undo"), fuzzy_score("undo", "Undo"));
        assert!(fuzzy_score("fit", "Fit / Zoom Extents").is_some());
    }

    #[test]
    fn prefix_match_outranks_mid_word_match() {
        let prefix = fuzzy_score("view", "View: Iso").unwrap();
        let mid = fuzzy_score("view", "Standard View: Iso").unwrap();
        assert!(
            prefix > mid,
            "prefix {prefix} should outrank mid-word {mid}"
        );
    }

    #[test]
    fn consecutive_run_outranks_scattered() {
        let tight = fuzzy_score("iso", "View: Iso").unwrap();
        let scattered = fuzzy_score("iso", "Solver Options: Invert Selection").unwrap();
        assert!(
            tight > scattered,
            "tight {tight} should outrank scattered {scattered}"
        );
    }

    #[test]
    fn longer_queries_score_higher_on_full_match() {
        let short = fuzzy_score("to", "Toggle Grid").unwrap();
        let long = fuzzy_score("toggle", "Toggle Grid").unwrap();
        assert!(long > short);
    }

    #[test]
    fn best_score_takes_the_maximum() {
        let best = best_score("grid", &["Toggle Ground Grid", "Node Labels"]);
        assert!(best.is_some());
        assert!(best_score("zzz", &["Toggle Ground Grid", "Node Labels"]).is_none());
        // The second field is the one that matches.
        assert!(best_score("label", &["Toggle Ground Grid", "Node Labels"]).is_some());
    }

    #[test]
    fn scoring_is_deterministic() {
        let a = fuzzy_score("sec", "Toggle Section Cut");
        let b = fuzzy_score("sec", "Toggle Section Cut");
        assert_eq!(a, b);
    }
}
