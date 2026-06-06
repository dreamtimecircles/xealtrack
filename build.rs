fn main() {
    // Only compile the windows resource file when targeting Windows
    if std::env::var("CARGO_CFG_TARGET_OS").unwrap() == "windows" {
        embed_resource::compile("resource.rc", embed_resource::NONE)
            .manifest_optional()
            .unwrap();
    }
}
