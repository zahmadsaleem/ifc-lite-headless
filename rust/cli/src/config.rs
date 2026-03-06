// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Conversion configuration, usable as both CLI args and library API.

/// How to name nodes in the output.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum ElementNaming {
    /// Use IFC GlobalId (22-char base64 GUID)
    #[default]
    Guids,
    /// Use IfcRoot.Name attribute
    Names,
    /// Use STEP entity id (#123)
    StepIds,
    /// Use IFC type name (e.g. IfcWall)
    Types,
}

/// Filter mode for include/exclude.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FilterMode {
    /// Match only the entity itself
    Direct,
    /// Match the entity and its decomposition/containment hierarchy
    Recursive,
}

/// A single include or exclude filter.
#[derive(Debug, Clone)]
pub struct EntityFilter {
    pub mode: FilterMode,
    /// IFC type names to match (uppercase, e.g. "IFCWALL")
    pub types: Vec<String>,
}

/// Verbosity level.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, PartialOrd, Ord)]
pub enum Verbosity {
    Quiet,
    #[default]
    Normal,
    Verbose,
}

/// Up axis for the output coordinate system.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum UpAxis {
    /// Z-up (IFC native, IfcConvert default)
    #[default]
    Z,
    /// Y-up (glTF spec default)
    Y,
}

/// Full conversion configuration.
#[derive(Debug, Clone)]
pub struct ConvertConfig {
    /// Include filters (if non-empty, only matching entities are included)
    pub include: Vec<EntityFilter>,
    /// Exclude filters (matching entities are excluded)
    pub exclude: Vec<EntityFilter>,
    /// How to name output nodes
    pub naming: ElementNaming,
    /// Up axis for output
    pub up_axis: UpAxis,
    /// Number of threads (0 = all cores)
    pub threads: usize,
    /// Verbosity level
    pub verbosity: Verbosity,
    /// Suppress progress bar
    pub no_progress: bool,
}

impl Default for ConvertConfig {
    fn default() -> Self {
        Self {
            include: Vec::new(),
            exclude: vec![EntityFilter {
                mode: FilterMode::Direct,
                types: vec!["IFCOPENINGELEMENT".into(), "IFCSPACE".into()],
            }],
            naming: ElementNaming::default(),
            up_axis: UpAxis::default(),
            threads: 0,
            verbosity: Verbosity::default(),
            no_progress: false,
        }
    }
}

impl ConvertConfig {
    /// Check if an entity type should be processed.
    pub fn should_process(&self, ifc_type_name: &str) -> bool {
        let upper = ifc_type_name.to_uppercase();

        // If include filters exist, entity must match at least one
        if !self.include.is_empty() {
            let included = self.include.iter().any(|f| f.types.iter().any(|t| t == &upper));
            if !included {
                return false;
            }
        }

        // Check exclude filters
        let excluded = self.exclude.iter().any(|f| f.types.iter().any(|t| t == &upper));
        !excluded
    }
}
