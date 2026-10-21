//! Exploration event log used for animated replay in the frontend.

use serde::Serialize;

pub const MAX_EVENTS: usize = 20_000;

#[derive(Clone, Debug, PartialEq, Serialize)]
#[serde(tag = "k", rename_all = "lowercase")]
pub enum Event {
    Expand { x: f64, y: f64, px: f64, py: f64 },
    Rewire { x: f64, y: f64, px: f64, py: f64, opx: f64, opy: f64 },
    Frontier { x: f64, y: f64 },
}

/// Event log capped at `MAX_EVENTS` (keeps the earliest).
#[derive(Clone, Debug, Default)]
pub struct EventLog {
    pub events: Vec<Event>,
    pub enabled: bool,
}

impl EventLog {
    pub fn new(enabled: bool) -> Self {
        Self { events: Vec::new(), enabled }
    }

    pub fn push(&mut self, e: Event) {
        if self.enabled && self.events.len() < MAX_EVENTS {
            self.events.push(e);
        }
    }
}

/// Raw planner result (before JSON shaping).
#[derive(Clone, Debug, Default)]
pub struct PlanOutput {
    pub found: bool,
    pub path: Vec<crate::geometry::Point>,
    pub nodes_explored: usize,
    pub iterations: usize,
}
