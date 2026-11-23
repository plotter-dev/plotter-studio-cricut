//! Materials and the Less/Default/More pressure model, matching the machine's dial.
//!
//! The dial has 16 positions: 1 = Custom, 2..=16 = fifteen stops from "Paper −" to
//! "Poster Board +". Each stop has a default blade pressure; Design Space's More/Less adjusts it
//! by a fixed step of 8 (measured on copy paper: 149 / 157 / 165).

use std::fmt;

/// Pressure adjustment, as Design Space's More/Default/Less dropdown.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum PressureAdjust {
    Less,
    #[default]
    Default,
    More,
}

impl PressureAdjust {
    pub fn offset(self) -> i16 {
        match self {
            PressureAdjust::Less => -8,
            PressureAdjust::Default => 0,
            PressureAdjust::More => 8,
        }
    }
}

/// One of the fifteen dial materials (including the "+"/"−" in-between stops).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Material {
    PaperMinus,
    Paper,
    PaperPlus,
    Vinyl,
    VinylPlus,
    IronOn,
    IronOnPlus,
    LightCardstock,
    LightCardstockPlus,
    Cardstock,
    CardstockPlus,
    BondedFabric,
    BondedFabricPlus,
    PosterBoard,
    PosterBoardPlus,
}

impl Material {
    /// All materials in dial order (dial position 2 first).
    pub const ALL: [Material; 15] = [
        Material::PaperMinus,
        Material::Paper,
        Material::PaperPlus,
        Material::Vinyl,
        Material::VinylPlus,
        Material::IronOn,
        Material::IronOnPlus,
        Material::LightCardstock,
        Material::LightCardstockPlus,
        Material::Cardstock,
        Material::CardstockPlus,
        Material::BondedFabric,
        Material::BondedFabricPlus,
        Material::PosterBoard,
        Material::PosterBoardPlus,
    ];

    pub fn name(self) -> &'static str {
        match self {
            Material::PaperMinus => "Paper −",
            Material::Paper => "Paper",
            Material::PaperPlus => "Paper +",
            Material::Vinyl => "Vinyl",
            Material::VinylPlus => "Vinyl +",
            Material::IronOn => "Iron-On",
            Material::IronOnPlus => "Iron-On +",
            Material::LightCardstock => "Light Cardstock",
            Material::LightCardstockPlus => "Light Cardstock +",
            Material::Cardstock => "Cardstock",
            Material::CardstockPlus => "Cardstock +",
            Material::BondedFabric => "Bonded Fabric",
            Material::BondedFabricPlus => "Bonded Fabric +",
            Material::PosterBoard => "Poster Board",
            Material::PosterBoardPlus => "Poster Board +",
        }
    }

    /// Default blade pressure (machine units).
    pub fn default_pressure(self) -> u16 {
        match self {
            Material::PaperMinus => 145,
            Material::Paper => 157,
            Material::PaperPlus => 168,
            Material::Vinyl => 180,
            Material::VinylPlus => 191,
            Material::IronOn => 203,
            Material::IronOnPlus => 214,
            Material::LightCardstock => 226,
            Material::LightCardstockPlus => 238,
            Material::Cardstock => 249,
            Material::CardstockPlus => 261,
            Material::BondedFabric => 272,
            Material::BondedFabricPlus => 284,
            Material::PosterBoard => 295,
            Material::PosterBoardPlus => 307,
        }
    }

    /// Pressure with a Less/Default/More adjustment.
    pub fn pressure(self, adjust: PressureAdjust) -> u16 {
        self.default_pressure()
            .saturating_add_signed(adjust.offset())
    }

    /// The dial position (2..=16) for this material.
    pub fn dial_position(self) -> u8 {
        // Variants are declared in dial order, so the discriminant is the stop index.
        self as u8 + 2
    }

    /// Look a material up by its display name (case-insensitive).
    pub fn from_name(name: &str) -> Option<Material> {
        Material::ALL
            .into_iter()
            .find(|m| m.name().eq_ignore_ascii_case(name.trim()))
    }
}

impl fmt::Display for Material {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// What the physical material dial is set to.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum DialSetting {
    /// The "Custom" position: the material must be chosen in software.
    Custom,
    /// One of the fifteen material stops.
    Material(Material),
    /// A value the machine reported that this crate doesn't recognise.
    Unknown(u8),
}

impl DialSetting {
    pub fn from_raw(raw: u8) -> DialSetting {
        match raw {
            1 => DialSetting::Custom,
            2..=16 => DialSetting::Material(Material::ALL[(raw - 2) as usize]),
            other => DialSetting::Unknown(other),
        }
    }

    pub fn material(self) -> Option<Material> {
        match self {
            DialSetting::Material(m) => Some(m),
            _ => None,
        }
    }
}

impl fmt::Display for DialSetting {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            DialSetting::Custom => f.write_str("Custom"),
            DialSetting::Material(m) => m.fmt(f),
            DialSetting::Unknown(v) => write!(f, "unknown dial value {v}"),
        }
    }
}

/// How a job picks its blade pressure.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MaterialSelection {
    /// Use whatever the machine's dial is set to (fails if the dial is on Custom).
    FromDial(PressureAdjust),
    /// Use this material regardless of the dial.
    Material(Material, PressureAdjust),
    /// Use a raw pressure value.
    Pressure(u16),
}

impl Default for MaterialSelection {
    fn default() -> Self {
        MaterialSelection::FromDial(PressureAdjust::Default)
    }
}

/// The material and pressure a job actually ended up with.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ResolvedMaterial {
    /// `None` when a raw pressure was given.
    pub material: Option<Material>,
    pub pressure: u16,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pressures_and_adjust() {
        assert_eq!(Material::Paper.pressure(PressureAdjust::Less), 149);
        assert_eq!(Material::Paper.pressure(PressureAdjust::Default), 157);
        assert_eq!(Material::Paper.pressure(PressureAdjust::More), 165);
        assert_eq!(Material::PosterBoardPlus.default_pressure(), 307);
    }

    #[test]
    fn dial_mapping() {
        assert_eq!(DialSetting::from_raw(1), DialSetting::Custom);
        assert_eq!(
            DialSetting::from_raw(2),
            DialSetting::Material(Material::PaperMinus)
        );
        assert_eq!(
            DialSetting::from_raw(3),
            DialSetting::Material(Material::Paper)
        );
        assert_eq!(
            DialSetting::from_raw(15),
            DialSetting::Material(Material::PosterBoard)
        );
        assert_eq!(
            DialSetting::from_raw(16),
            DialSetting::Material(Material::PosterBoardPlus)
        );
        assert_eq!(DialSetting::from_raw(0), DialSetting::Unknown(0));
        for m in Material::ALL {
            assert_eq!(
                DialSetting::from_raw(m.dial_position()),
                DialSetting::Material(m)
            );
        }
        assert_eq!(Material::from_name("vinyl"), Some(Material::Vinyl));
    }
}
