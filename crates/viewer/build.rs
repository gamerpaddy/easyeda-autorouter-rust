use std::process::Command;

fn main() {
    for path in ["build.rs", "src", "Cargo.toml", "../router/src", "../dsn/src", "../cli/src", "../../Cargo.lock"] {
        println!("cargo:rerun-if-changed={path}");
    }
    let output = if cfg!(windows) {
        Command::new("powershell.exe")
            .args(["-NoProfile", "-NonInteractive", "-Command", "[DateTime]::UtcNow.ToString('yyyy-MM-dd HH:mm:ss')"])
            .output()
    } else {
        Command::new("date").args(["-u", "+%Y-%m-%d %H:%M:%S"]).output()
    };
    let stamp = output.ok().filter(|o| o.status.success())
        .and_then(|o| String::from_utf8(o.stdout).ok())
        .map(|s| s.trim().to_owned()).filter(|s| !s.is_empty());
    println!("cargo:rustc-env=AUTOROUTER_BUILD_DATE={}",
        stamp.map_or_else(|| "unknown".into(), |s| format!("{s} UTC")));
}
