//! `krl-parser-smoke <bt2krl output dir>`: run the parser smoke check on generated KRL.

fn main() -> std::process::ExitCode {
    let Some(dir) = std::env::args().nth(1) else {
        eprintln!("usage: krl-parser-smoke <bt2krl output dir>");
        return std::process::ExitCode::from(2);
    };
    match krl_parser_smoke::check_dir(std::path::Path::new(&dir)) {
        Ok(lines) => {
            for l in lines {
                println!("{l}");
            }
            std::process::ExitCode::SUCCESS
        }
        Err(e) => {
            eprintln!("error: {e}");
            std::process::ExitCode::from(e.exit_code() as u8)
        }
    }
}
