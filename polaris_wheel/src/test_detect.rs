fn main() {
    let gilrs = gilrs::Gilrs::new().unwrap();
    let count = gilrs.gamepads().count();
    println!("Gamepads found: {}", count);
    for (_id, gp) in gilrs.gamepads() {
        println!("  {} (mapping: {:?})", gp.name(), gp.mapping_source());
    }
}
