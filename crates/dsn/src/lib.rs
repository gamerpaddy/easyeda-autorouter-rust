//! Specctra DSN parser (own implementation) producing a typed `Board`.

pub mod easyeda;
pub mod easyeda_pro;

use std::collections::HashMap;

// ---------------------------------------------------------------- s-expr
#[derive(Debug, Clone)]
pub enum Node {
    Atom(String),
    List(Vec<Node>),
}

impl Node {
    pub fn atom(&self) -> Option<&str> {
        match self {
            Node::Atom(s) => Some(s),
            _ => None,
        }
    }
    pub fn list(&self) -> Option<&[Node]> {
        match self {
            Node::List(v) => Some(v),
            _ => None,
        }
    }
    /// head keyword of a list
    pub fn head(&self) -> Option<&str> {
        self.list().and_then(|l| l.first()).and_then(|n| n.atom())
    }
    pub fn num(&self) -> Option<f64> {
        self.atom().and_then(|s| s.parse().ok())
    }
}

pub fn parse_sexpr(src: &str) -> Result<Node, String> {
    let b: Vec<char> = src.chars().collect();
    let mut i = 0;
    let mut quote = '"';
    let mut stack: Vec<Vec<Node>> = vec![vec![]];
    while i < b.len() {
        let c = b[i];
        match c {
            '(' => {
                stack.push(vec![]);
                i += 1;
            }
            ')' => {
                let done = stack.pop().ok_or("unbalanced )")?;
                stack
                    .last_mut()
                    .ok_or("unbalanced )")?
                    .push(Node::List(done));
                i += 1;
            }
            c if c.is_whitespace() => i += 1,
            c if c == quote => {
                let mut s = String::new();
                i += 1;
                while i < b.len() && b[i] != quote {
                    s.push(b[i]);
                    i += 1;
                }
                i += 1;
                stack.last_mut().unwrap().push(Node::Atom(s));
            }
            _ => {
                let mut s = String::new();
                while i < b.len() && !b[i].is_whitespace() && b[i] != '(' && b[i] != ')' {
                    s.push(b[i]);
                    i += 1;
                }
                // (string_quote ") declaration
                if s == "string_quote" {
                    let mut j = i;
                    while j < b.len() && b[j].is_whitespace() {
                        j += 1;
                    }
                    if j < b.len() && b[j] != ')' {
                        quote = b[j];
                    }
                }
                stack.last_mut().unwrap().push(Node::Atom(s));
            }
        }
    }
    if stack.len() != 1 {
        return Err("unbalanced (".into());
    }
    let mut top = stack.pop().unwrap();
    top.pop().ok_or_else(|| "empty file".to_string())
}

// ---------------------------------------------------------------- model
#[derive(Debug, Clone)]
pub enum Shape {
    Circle {
        layer: String,
        d: f64,
        cx: f64,
        cy: f64,
    },
    Rect {
        layer: String,
        x1: f64,
        y1: f64,
        x2: f64,
        y2: f64,
    },
    Polygon {
        layer: String,
        aperture: f64,
        pts: Vec<(f64, f64)>,
    },
    Path {
        layer: String,
        aperture: f64,
        pts: Vec<(f64, f64)>,
    },
}

impl Shape {
    pub fn layer(&self) -> &str {
        match self {
            Shape::Circle { layer, .. }
            | Shape::Rect { layer, .. }
            | Shape::Polygon { layer, .. }
            | Shape::Path { layer, .. } => layer,
        }
    }
}

#[derive(Debug, Clone, Default)]
pub struct Padstack {
    pub name: String,
    pub shapes: Vec<Shape>,
}

#[derive(Debug, Clone)]
pub struct ImagePin {
    pub padstack: String,
    pub id: String,
    pub x: f64,
    pub y: f64,
    pub rot: f64,
}

#[derive(Debug, Clone, Default)]
pub struct Image {
    pub name: String,
    pub pins: Vec<ImagePin>,
    pub keepouts: Vec<Keepout>,
}

#[derive(Debug, Clone)]
pub struct Placed {
    pub reference: String,
    pub image: String,
    pub x: f64,
    pub y: f64,
    pub back: bool,
    pub rot: f64,
}

#[derive(Debug, Clone)]
pub struct Keepout {
    pub layer: Option<String>, // None = all layers
    pub shape: Shape,
    pub via_only: bool,
    pub wire_only: bool,
}

#[derive(Debug, Clone)]
pub struct Layer {
    pub name: String,
    pub power: bool, // (type power) => treated as exclusive plane
}

#[derive(Debug, Clone)]
pub struct PinRef {
    pub comp: String,
    pub pin: String,
}

#[derive(Debug, Clone)]
pub struct Net {
    pub name: String,
    pub pins: Vec<PinRef>,
}

#[derive(Debug, Clone, Default)]
pub struct Class {
    pub name: String,
    pub nets: Vec<String>,
    pub width: Option<f64>,
    pub clearance: Option<f64>,
    pub vias: Vec<String>,
}

/// Pin with its resolved absolute geometry.
#[derive(Debug, Clone)]
pub struct AbsPin {
    pub comp: String,
    pub pin: String,
    pub x: f64,
    pub y: f64,
    /// shapes in absolute coordinates, layer = layer *index*
    pub shapes: Vec<(usize, AbsShape)>,
}

#[derive(Debug, Clone)]
pub enum AbsShape {
    Circle { cx: f64, cy: f64, r: f64 },
    Poly(Vec<(f64, f64)>),
}

/// A trace already present in the DSN `(wiring)` section.
#[derive(Debug, Clone)]
pub struct DsnWire {
    pub net: String,
    pub layer: String,
    pub width: f64,
    pub pts: Vec<(f64, f64)>,
    pub kind: String,
}

#[derive(Debug, Clone)]
pub struct DsnVia {
    pub name: String,
    pub x: f64,
    pub y: f64,
    pub net: String,
}

/// A copper plane `(plane NET (polygon LAYER 0 x y ...))`.
#[derive(Debug, Clone)]
pub struct Plane {
    pub net: String,
    pub layer: String,
    pub pts: Vec<(f64, f64)>,
}

#[derive(Debug, Clone, Default)]
pub struct Board {
    pub wires: Vec<DsnWire>,
    pub vias: Vec<DsnVia>,
    pub planes: Vec<Plane>,
    pub name: String,
    pub res_unit: String,
    pub res: f64,
    pub boundary: Vec<(f64, f64)>,
    pub layers: Vec<Layer>,
    pub via_names: Vec<String>,
    pub grid_wire: f64,
    pub rule_clear: f64,
    pub rule_width: f64,
    pub padstacks: HashMap<String, Padstack>,
    pub images: HashMap<String, Image>,
    pub components: Vec<Placed>,
    pub nets: Vec<Net>,
    pub classes: Vec<Class>,
    pub keepouts: Vec<Keepout>,
}

impl Board {
    /// Millimetres per coordinate unit as literally declared by `(resolution unit n)`.
    pub fn declared_unit_mm(&self) -> f64 {
        let u = match self.res_unit.to_lowercase().as_str() {
            "mil" => 0.0254,
            "inch" | "in" => 25.4,
            "mm" => 1.0,
            "cm" => 10.0,
            "um" | "micron" => 0.001,
            _ => 0.0254,
        };
        // `(resolution mil 10)` means 10 coordinate counts per mil.
        u / self.res.max(1e-12)
    }

    /// Millimetres per coordinate unit, corrected when the declaration is physically absurd
    /// (several EDA exporters write `mil 1000` while the numbers are really 10 mil units).
    /// Candidates are accepted when the default clearance, width and board size are plausible.
    pub fn detect_unit_mm(&self) -> (f64, bool) {
        let (mut w, mut h) = (0.0f64, 0.0f64);
        if !self.boundary.is_empty() {
            let xs = self.boundary.iter().map(|p| p.0);
            let ys = self.boundary.iter().map(|p| p.1);
            w = xs.clone().fold(f64::MIN, f64::max) - xs.fold(f64::MAX, f64::min);
            h = ys.clone().fold(f64::MIN, f64::max) - ys.fold(f64::MAX, f64::min);
        }
        let size = w.max(h);
        let plausible = |u: f64| {
            let clr_ok = self.rule_clear <= 0.0 || (0.04..=1.5).contains(&(self.rule_clear * u));
            let wid_ok = self.rule_width <= 0.0 || (0.04..=3.0).contains(&(self.rule_width * u));
            let size_ok = size <= 0.0 || (8.0..=1500.0).contains(&(size * u));
            clr_ok && wid_ok && size_ok
        };
        let declared = self.declared_unit_mm();
        if plausible(declared) {
            return (declared, false);
        }
        for u in [0.0254, 0.254, 0.1, 1.0, 0.00254] {
            if plausible(u) {
                return (u, true);
            }
        }
        (declared, false)
    }

    pub fn layer_index(&self, name: &str) -> Option<usize> {
        self.layers.iter().position(|l| l.name == name)
    }

    /// Via padstack: radius on each layer (index by layer) – uses the first via name.
    pub fn via_padstack(&self, name: &str) -> Option<&Padstack> {
        self.padstacks.get(name)
    }

    /// All pins with absolute geometry, keyed "comp-pin".
    pub fn abs_pins(&self) -> HashMap<String, AbsPin> {
        let mut out = HashMap::new();
        for c in &self.components {
            let Some(img) = self.images.get(&c.image) else {
                continue;
            };
            for p in &img.pins {
                let (lx, ly) = (p.x, p.y);
                let (px, py) = place(lx, ly, c);
                let Some(ps) = self.padstacks.get(&p.padstack) else {
                    continue;
                };
                let mut shapes = vec![];
                for s in &ps.shapes {
                    let Some(mut li) = self.layer_index(s.layer()) else {
                        continue;
                    };
                    if c.back {
                        li = self.layers.len() - 1 - li;
                    }
                    let tr = |x: f64, y: f64| {
                        // pad-local -> image (pin rot) -> placed
                        let (x, y) = rot(x, y, p.rot);
                        place(lx + x, ly + y, c)
                    };
                    match s {
                        Shape::Circle { d, cx, cy, .. } => {
                            let (x, y) = tr(*cx, *cy);
                            shapes.push((
                                li,
                                AbsShape::Circle {
                                    cx: x,
                                    cy: y,
                                    r: d / 2.0,
                                },
                            ));
                        }
                        Shape::Rect { x1, y1, x2, y2, .. } => {
                            let pts = vec![(*x1, *y1), (*x2, *y1), (*x2, *y2), (*x1, *y2)];
                            shapes.push((
                                li,
                                AbsShape::Poly(pts.iter().map(|&(x, y)| tr(x, y)).collect()),
                            ));
                        }
                        Shape::Polygon { pts, .. } | Shape::Path { pts, .. } => {
                            shapes.push((
                                li,
                                AbsShape::Poly(pts.iter().map(|&(x, y)| tr(x, y)).collect()),
                            ));
                        }
                    }
                }
                out.insert(
                    format!("{}-{}", c.reference, p.id),
                    AbsPin {
                        comp: c.reference.clone(),
                        pin: p.id.clone(),
                        x: px,
                        y: py,
                        shapes,
                    },
                );
            }
        }
        out
    }
}

fn rot(x: f64, y: f64, deg: f64) -> (f64, f64) {
    if deg == 0.0 {
        return (x, y);
    }
    let (s, c) = deg.to_radians().sin_cos();
    (x * c - y * s, x * s + y * c)
}

fn place(x: f64, y: f64, c: &Placed) -> (f64, f64) {
    let x = if c.back { -x } else { x };
    let (x, y) = rot(x, y, c.rot);
    (x + c.x, y + c.y)
}

// ---------------------------------------------------------------- parse
fn nums(nodes: &[Node]) -> Vec<f64> {
    nodes.iter().filter_map(|n| n.num()).collect()
}
fn pairs(v: &[f64]) -> Vec<(f64, f64)> {
    v.chunks_exact(2).map(|c| (c[0], c[1])).collect()
}

fn parse_shape(n: &Node) -> Option<Shape> {
    let l = n.list()?;
    let kind = l.first()?.atom()?;
    let layer = l.get(1)?.atom()?.to_string();
    let v = nums(&l[2..]);
    match kind {
        "circle" => Some(Shape::Circle {
            layer,
            d: *v.first()?,
            cx: v.get(1).copied().unwrap_or(0.0),
            cy: v.get(2).copied().unwrap_or(0.0),
        }),
        "rect" | "rectangle" => {
            if v.len() < 4 {
                return None;
            }
            Some(Shape::Rect {
                layer,
                x1: v[0].min(v[2]),
                y1: v[1].min(v[3]),
                x2: v[0].max(v[2]),
                y2: v[1].max(v[3]),
            })
        }
        "polygon" => Some(Shape::Polygon {
            layer,
            aperture: *v.first()?,
            pts: pairs(&v[1..]),
        }),
        "path" => Some(Shape::Path {
            layer,
            aperture: *v.first()?,
            pts: pairs(&v[1..]),
        }),
        _ => None,
    }
}

fn parse_keepout(n: &Node) -> Option<Keepout> {
    let l = n.list()?;
    let mut shape = None;
    let mut via_only = false;
    let mut wire_only = false;
    for c in &l[1..] {
        match c.head() {
            Some("circle") | Some("rect") | Some("polygon") | Some("path") | Some("rectangle") => {
                shape = parse_shape(c)
            }
            Some("via_keepout") => via_only = true,
            Some("wire_keepout") => wire_only = true,
            _ => {}
        }
    }
    let shape = shape?;
    let layer = match shape.layer() {
        "signal" | "pcb" => None,
        s => Some(s.to_string()),
    };
    Some(Keepout {
        layer,
        shape,
        via_only,
        wire_only,
    })
}

pub fn parse_dsn(src: &str) -> Result<Board, String> {
    let root = parse_sexpr(src)?;
    let items = root.list().ok_or("bad root")?;
    if !matches!(items.first().and_then(|n| n.atom()), Some(s) if s.eq_ignore_ascii_case("pcb")) {
        return Err("not a DSN file (missing pcb)".into());
    }
    let mut b = Board {
        res_unit: "mil".into(),
        res: 10.0,
        grid_wire: 0.0,
        ..Default::default()
    };
    b.name = items
        .get(1)
        .and_then(|n| n.atom())
        .unwrap_or("")
        .to_string();

    for it in &items[1..] {
        match it.head() {
            Some("resolution") => {
                let l = it.list().unwrap();
                b.res_unit = l.get(1).and_then(|n| n.atom()).unwrap_or("mil").to_string();
                b.res = l.get(2).and_then(|n| n.num()).unwrap_or(10.0);
            }
            Some("structure") => parse_structure(it, &mut b),
            Some("placement") => {
                for c in it.list().unwrap() {
                    if c.head() != Some("component") {
                        continue;
                    }
                    let l = c.list().unwrap();
                    let image = l[1].atom().unwrap_or("").to_string();
                    for p in &l[2..] {
                        if p.head() != Some("place") {
                            continue;
                        }
                        let pl = p.list().unwrap();
                        let reference = pl.get(1).and_then(|n| n.atom()).unwrap_or("").to_string();
                        let x = pl.get(2).and_then(|n| n.num()).unwrap_or(0.0);
                        let y = pl.get(3).and_then(|n| n.num()).unwrap_or(0.0);
                        let back = pl.get(4).and_then(|n| n.atom()) == Some("back");
                        let r = pl.get(5).and_then(|n| n.num()).unwrap_or(0.0);
                        b.components.push(Placed {
                            reference,
                            image: image.clone(),
                            x,
                            y,
                            back,
                            rot: r,
                        });
                    }
                }
            }
            Some("library") => {
                for c in it.list().unwrap() {
                    match c.head() {
                        Some("image") => {
                            let l = c.list().unwrap();
                            let mut img = Image {
                                name: l[1].atom().unwrap_or("").into(),
                                ..Default::default()
                            };
                            for p in &l[2..] {
                                match p.head() {
                                    Some("pin") => {
                                        let pl = p.list().unwrap();
                                        let padstack = pl[1].atom().unwrap_or("").to_string();
                                        let mut k = 2;
                                        let mut r = 0.0;
                                        if pl.get(k).and_then(|n| n.head()) == Some("rotate") {
                                            r = pl[k].list().unwrap()[1].num().unwrap_or(0.0);
                                            k += 1;
                                        }
                                        let id = pl
                                            .get(k)
                                            .and_then(|n| n.atom())
                                            .unwrap_or("")
                                            .to_string();
                                        let x = pl.get(k + 1).and_then(|n| n.num()).unwrap_or(0.0);
                                        let y = pl.get(k + 2).and_then(|n| n.num()).unwrap_or(0.0);
                                        img.pins.push(ImagePin {
                                            padstack,
                                            id,
                                            x,
                                            y,
                                            rot: r,
                                        });
                                    }
                                    Some("keepout") | Some("via_keepout") => {
                                        if let Some(k) = parse_keepout(p) {
                                            img.keepouts.push(k)
                                        }
                                    }
                                    _ => {}
                                }
                            }
                            b.images.insert(img.name.clone(), img);
                        }
                        Some("padstack") => {
                            let l = c.list().unwrap();
                            let mut ps = Padstack {
                                name: l[1].atom().unwrap_or("").into(),
                                shapes: vec![],
                            };
                            for s in &l[2..] {
                                if s.head() == Some("shape") {
                                    if let Some(sh) = s.list().unwrap().get(1).and_then(parse_shape)
                                    {
                                        ps.shapes.push(sh);
                                    }
                                }
                            }
                            b.padstacks.insert(ps.name.clone(), ps);
                        }
                        _ => {}
                    }
                }
            }
            Some("wiring") => {
                for c in it.list().unwrap() {
                    match c.head() {
                        Some("wire") => {
                            let l = c.list().unwrap();
                            let (mut net, mut kind) = (String::new(), String::from("route"));
                            let mut path: Option<Shape> = None;
                            for q in &l[1..] {
                                match q.head() {
                                    Some("path") | Some("polygon") | Some("rect") => {
                                        path = parse_shape(q)
                                    }
                                    Some("shape") => {
                                        if let Some(inner) = q.list().unwrap().get(1) {
                                            path = parse_shape(inner);
                                        }
                                    }
                                    Some("net") => {
                                        net = q
                                            .list()
                                            .unwrap()
                                            .get(1)
                                            .and_then(|n| n.atom())
                                            .unwrap_or("")
                                            .to_string()
                                    }
                                    Some("type") => {
                                        kind = q
                                            .list()
                                            .unwrap()
                                            .get(1)
                                            .and_then(|n| n.atom())
                                            .unwrap_or("route")
                                            .to_string()
                                    }
                                    _ => {}
                                }
                            }
                            if let Some(Shape::Path {
                                layer,
                                aperture,
                                pts,
                            }) = path
                            {
                                b.wires.push(DsnWire {
                                    net,
                                    layer,
                                    width: aperture,
                                    pts,
                                    kind,
                                });
                            } else if let Some(shape) = path {
                                if let Some((layer, pts)) = plane_shape(shape) {
                                    b.planes.push(Plane { net, layer, pts });
                                }
                            }
                        }
                        Some("via") => {
                            let l = c.list().unwrap();
                            let name = l.get(1).and_then(|n| n.atom()).unwrap_or("").to_string();
                            let x = l.get(2).and_then(|n| n.num()).unwrap_or(0.0);
                            let y = l.get(3).and_then(|n| n.num()).unwrap_or(0.0);
                            let mut net = String::new();
                            for q in &l[4..] {
                                if q.head() == Some("net") {
                                    net = q
                                        .list()
                                        .unwrap()
                                        .get(1)
                                        .and_then(|n| n.atom())
                                        .unwrap_or("")
                                        .to_string();
                                }
                            }
                            b.vias.push(DsnVia { name, x, y, net });
                        }
                        _ => {}
                    }
                }
            }
            Some("network") => {
                for c in it.list().unwrap() {
                    match c.head() {
                        Some("net") => {
                            let l = c.list().unwrap();
                            let mut net = Net {
                                name: l[1].atom().unwrap_or("").into(),
                                pins: vec![],
                            };
                            for p in &l[2..] {
                                if matches!(p.head(), Some("pins") | Some("pin")) {
                                    for t in &p.list().unwrap()[1..] {
                                        if let Some(s) = t.atom() {
                                            if let Some((cmp, pin)) = split_pin(s, &b.components) {
                                                net.pins.push(PinRef { comp: cmp, pin });
                                            }
                                        }
                                    }
                                }
                            }
                            b.nets.push(net);
                        }
                        Some("class") => {
                            let l = c.list().unwrap();
                            let mut cl = Class {
                                name: l[1].atom().unwrap_or("").into(),
                                ..Default::default()
                            };
                            for p in &l[2..] {
                                match p {
                                    Node::Atom(s) => cl.nets.push(s.clone()),
                                    Node::List(ll) => match p.head() {
                                        Some("circuit") => {
                                            for q in &ll[1..] {
                                                if q.head() == Some("use_via") {
                                                    for v in &q.list().unwrap()[1..] {
                                                        if let Some(s) = v.atom() {
                                                            cl.vias.push(s.to_string());
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                        Some("rule") => {
                                            for q in &ll[1..] {
                                                match q.head() {
                                                    Some("width") => {
                                                        cl.width = q
                                                            .list()
                                                            .unwrap()
                                                            .get(1)
                                                            .and_then(|n| n.num())
                                                    }
                                                    Some("clearance") | Some("clear") => {
                                                        cl.clearance = q
                                                            .list()
                                                            .unwrap()
                                                            .get(1)
                                                            .and_then(|n| n.num())
                                                    }
                                                    _ => {}
                                                }
                                            }
                                        }
                                        _ => {}
                                    },
                                }
                            }
                            b.classes.push(cl);
                        }
                        _ => {}
                    }
                }
            }
            _ => {}
        }
    }
    if b.boundary.is_empty() {
        return Err("no boundary".into());
    }
    for pl in &b.planes {
        if !b.nets.iter().any(|n| n.name == pl.net) {
            b.nets.push(Net {
                name: pl.net.clone(),
                pins: vec![],
            });
        }
    }
    if b.layers.is_empty() {
        return Err("no layers".into());
    }
    if let Some(unit) = items.iter().find(|it| it.head() == Some("unit")) {
        b.res_unit = unit
            .list()
            .and_then(|l| l.get(1))
            .and_then(|a| a.atom())
            .ok_or("invalid unit")?
            .to_string();
    }
    if b.boundary.len() < 3 || b.layers.is_empty() {
        return Err("DSN requires a board outline and copper layers".into());
    }
    if !b.res.is_finite() || b.res <= 0.0 {
        return Err("invalid DSN resolution".into());
    }
    Ok(b)
}

fn split_pin(s: &str, comps: &[Placed]) -> Option<(String, String)> {
    // try every '-' as separator; prefer one where the prefix is a known component
    let idx: Vec<usize> = s.match_indices('-').map(|(i, _)| i).collect();
    for &i in &idx {
        if comps.iter().any(|c| c.reference == s[..i]) {
            return Some((s[..i].to_string(), s[i + 1..].to_string()));
        }
    }
    idx.last()
        .map(|&i| (s[..i].to_string(), s[i + 1..].to_string()))
}

fn parse_structure(st: &Node, b: &mut Board) {
    for c in &st.list().unwrap()[1..] {
        match c.head() {
            Some("boundary") => {
                for p in &c.list().unwrap()[1..] {
                    if p.head() == Some("path") || p.head() == Some("polygon") {
                        if let Some(Shape::Path { pts, .. }) | Some(Shape::Polygon { pts, .. }) =
                            parse_shape(p)
                        {
                            if b.boundary.is_empty() {
                                b.boundary = pts;
                            }
                        } else if let Some(sh) = parse_shape(p) {
                            let _ = sh;
                        }
                    } else if p.head() == Some("rect") {
                        if let Some(Shape::Rect { x1, y1, x2, y2, .. }) = parse_shape(p) {
                            b.boundary = vec![(x1, y1), (x2, y1), (x2, y2), (x1, y2)];
                        }
                    }
                }
            }
            Some("layer") => {
                let l = c.list().unwrap();
                let name = l[1].atom().unwrap_or("").to_string();
                let mut power = false;
                for q in &l[2..] {
                    if q.head() == Some("type") {
                        if let Some(t) = q.list().unwrap().get(1).and_then(|n| n.atom()) {
                            power = t == "power";
                        }
                    }
                }
                b.layers.push(Layer { name, power });
            }
            Some("via") => {
                for v in &c.list().unwrap()[1..] {
                    if let Some(s) = v.atom() {
                        b.via_names.push(s.to_string());
                    }
                }
            }
            Some("grid") => {
                let l = c.list().unwrap();
                if l.get(1).and_then(|n| n.atom()) == Some("wire") {
                    b.grid_wire = l.get(2).and_then(|n| n.num()).unwrap_or(0.0);
                }
            }
            Some("rule") => {
                for q in &c.list().unwrap()[1..] {
                    match q.head() {
                        Some("width") => {
                            b.rule_width = q
                                .list()
                                .unwrap()
                                .get(1)
                                .and_then(|n| n.num())
                                .unwrap_or(0.0)
                        }
                        Some("clear") | Some("clearance") => {
                            // only the default (no (type ..)) one counts as the global
                            let l = q.list().unwrap();
                            if l.len() == 2 {
                                b.rule_clear = l[1].num().unwrap_or(0.0);
                            }
                        }
                        _ => {}
                    }
                }
            }
            Some("plane") => {
                let l = c.list().unwrap();
                let net = l.get(1).and_then(|n| n.atom()).unwrap_or("").to_string();
                for q in &l[2..] {
                    if matches!(q.head(), Some("polygon") | Some("rect")) {
                        if let Some((layer, pts)) = parse_shape(q).and_then(plane_shape) {
                            b.planes.push(Plane {
                                net: net.clone(),
                                layer,
                                pts,
                            });
                        }
                    }
                }
            }
            Some("keepout") | Some("via_keepout") | Some("wire_keepout") => {
                if let Some(k) = parse_keepout(c) {
                    b.keepouts.push(k);
                }
            }
            _ => {}
        }
    }
}

// ---------------------------------------------------------------- pairs
fn plane_shape(shape: Shape) -> Option<(String, Vec<(f64, f64)>)> {
    match shape {
        Shape::Polygon { layer, pts, .. } => Some((layer, pts)),
        Shape::Rect {
            layer,
            x1,
            y1,
            x2,
            y2,
        } => Some((layer, vec![(x1, y1), (x2, y1), (x2, y2), (x1, y2)])),
        _ => None,
    }
}
/// Differential pair detection: a net ending in `+` and one ending in `-` with the same
/// non-empty base name. `+12V` / `-12V` (prefix polarity) are never pairs. Names like `KEY_P`/`KEY_N` are not pairs either.
pub fn find_diff_pairs(nets: &[Net]) -> Vec<(usize, usize)> {
    let mut pos: HashMap<String, usize> = HashMap::new();
    let mut neg: HashMap<String, usize> = HashMap::new();
    for (i, n) in nets.iter().enumerate() {
        let upper = n.name.to_uppercase();
        let s = upper.as_str();
        // Button polarity names commonly use P/N without representing data pairs.
        if s.starts_with("KEY_") || s.starts_with("BUTTON_") {
            continue;
        }
        if s.len() > 1 {
            if let Some(b) = s.strip_suffix('+') {
                pos.insert(b.to_string(), i);
            } else if let Some(b) = s.strip_suffix('-') {
                neg.insert(b.to_string(), i);
            } else if let Some(b) = s.strip_suffix("_P") {
                pos.insert(format!("{b}_"), i);
            } else if let Some(b) = s.strip_suffix("_N") {
                neg.insert(format!("{b}_"), i);
            } else if let Some(b) = s.strip_suffix("DP") {
                pos.insert(format!("{b}D"), i);
            } else if let Some(b) = s.strip_suffix("DN") {
                neg.insert(format!("{b}D"), i);
            }
        }
    }
    let mut v: Vec<(usize, usize)> = pos
        .iter()
        .filter_map(|(b, &p)| neg.get(b).map(|&n| (p, n)))
        .collect();
    v.sort();
    v
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pairs() {
        let mk = |n: &str| Net {
            name: n.into(),
            pins: vec![],
        };
        let nets = vec![
            mk("+12V"),
            mk("-12V"),
            mk("CLK+"),
            mk("CLK-"),
            mk("D_P"),
            mk("D_N"),
            mk("X+"),
        ];
        let p = find_diff_pairs(&nets);
        assert_eq!(p, vec![(2, 3), (4, 5)]);
    }

    #[test]
    fn sample() {
        let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../Autorouter_PCB_Curvetracer1_2026-11-15.dsn");
        let Ok(s) = std::fs::read_to_string(p) else {
            return;
        };
        let b = parse_dsn(&s).unwrap();
        assert_eq!(b.layers.len(), 4);
        assert!(b.nets.len() > 40);
        let pins = b.abs_pins();
        for n in &b.nets {
            for p in &n.pins {
                assert!(
                    pins.contains_key(&format!("{}-{}", p.comp, p.pin)),
                    "{:?}",
                    p
                );
            }
        }
    }
}
