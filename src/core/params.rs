/// Global simulation parameters that should be consistent across broad-phase, narrow-phase,
/// and the solver.
///
/// Keep this small and explicit: we only put "cross-cutting" knobs here.
#[derive(Debug, Clone, Copy)]
pub struct SimParams {
    /// Extra distance to treat as "in range" for speculative contacts.
    ///
    /// Used to:
    /// - expand broad-phase AABBs (fat AABB)
    /// - allow narrow-phase to emit contacts slightly before overlap
    pub speculative_distance: f32,
}

impl Default for SimParams {
    fn default() -> Self {
        Self {
            speculative_distance: 0.05,
        }
    }
}
