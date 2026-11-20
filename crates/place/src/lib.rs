pub mod draw;
pub mod engine;
pub mod geom;
pub mod model;
pub mod pro_fmt;
pub mod std_fmt;

use model::*;

pub enum Doc {
    Std(std_fmt::StdDoc),
    Pro(pro_fmt::ProDoc),
}

/// Load an EasyEDA Standard `.json` or Pro `.epro2` file (the content decides, not the name).
pub fn load(bytes: &[u8]) -> Result<(Doc, Board), String> {
    if bytes.starts_with(b"PK") {
        let (d, b) = pro_fmt::load(bytes)?;
        Ok((Doc::Pro(d), b))
    } else {
        let (d, b) = std_fmt::load(bytes)?;
        Ok((Doc::Std(d), b))
    }
}

impl Doc {
    /// The file with the placement applied. With `clear_routing` the tracks and vias that were on
    /// the board are removed (they no longer fit the moved parts).
    pub fn save(&self, board: &Board, pls: &[Pl], clear_routing: bool, outline: Option<[f64; 4]>) -> Result<Vec<u8>, String> {
        match self {
            Doc::Std(d) => match outline {
                Some(r) => d.with_outline(r).save(board, pls, clear_routing),
                None => d.save(board, pls, clear_routing),
            },
            Doc::Pro(d) => d.save(board, pls, clear_routing),
        }
    }

    pub fn can_write_outline(&self) -> bool {
        matches!(self, Doc::Std(_))
    }

    pub fn ext(&self) -> &'static str {
        match self {
            Doc::Std(_) => "json",
            Doc::Pro(_) => "epro2",
        }
    }
}
