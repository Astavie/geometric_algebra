use std::{
    fs::{create_dir_all, File},
    io::Write,
    path::Path,
};

use codegen;

fn main() {
    let algebras = [
        "epga1d:1,1;Scalar:1;ComplexNumber:1,e01",
        "ppga1d:0,1;Scalar:1;DualNumber:1,e01",
        "hpga1d:-1,1;Scalar:1;SplitComplexNumber:1,e01",
        "epga2d:1,1,1;Scalar:1;PseudoScalar:e012;Rotor:1,e12;Point:e12,e01,-e02;Direction:e01,-e02;Line:e0,e2,e1;Translator:1,e01,-e02;Motor:1,e12,e01,-e02;MotorDual:e012,e0,e2,e1",
        "ppga2d:0,1,1;Scalar:1;PseudoScalar:e012;Rotor:1,e12;Point:e12,e01,-e02;Direction:e01,-e02;Line:e0,e2,e1;Translator:1,e01,-e02;Motor:1,e12,e01,-e02;MotorDual:e012,e0,e2,e1",
        "hpga2d:-1,1,1;Scalar:1;PseudoScalar:e012;Rotor:1,e12;Point:e12,e01,-e02;Direction:e01,-e02;Line:e0,e2,e1;Translator:1,e01,-e02;Motor:1,e12,e01,-e02;MotorDual:e012,e0,e2,e1",
        "epga3d:1,1,1,1;Scalar:1;PseudoScalar:e0123;Rotor:1,e23,-e13,e12;Origin:e123;Point:e123,-e023,e013,-e012;Direction:-e023,e013,-e012;IdealLine:e01,e02,e03;Branch:e23,-e13,e12;Plane:e0,e1,e2,e3;Line:e01,e02,e03|e23,-e13,e12;Translator:1,e01,e02,e03;Motor:1,e23,-e13,e12|e0123,e01,e02,e03",
        "ppga3d:0,1,1,1;Scalar:1;PseudoScalar:e0123;Rotor:1,e23,-e13,e12;Origin:e123;Point:e123,-e023,e013,-e012;Direction:-e023,e013,-e012;IdealLine:e01,e02,e03;Branch:e23,-e13,e12;Plane:e0,e1,e2,e3;Line:e01,e02,e03|e23,-e13,e12;Translator:1,e01,e02,e03;Motor:1,e23,-e13,e12|e0123,e01,e02,e03",
        "hpga3d:-1,1,1,1;Scalar:1;PseudoScalar:e0123;Rotor:1,e23,-e13,e12;Origin:e123;Point:e123,-e023,e013,-e012;Direction:-e023,e013,-e012;IdealLine:e01,e02,e03;Branch:e23,-e13,e12;Plane:e0,e1,e2,e3;Line:e01,e02,e03|e23,-e13,e12;Translator:1,e01,e02,e03;Motor:1,e23,-e13,e12|e0123,e01,e02,e03",
    ];

    let dir = env!("CARGO_MANIFEST_DIR");
    let dir = format!("{}/src/generated", dir);

    create_dir_all(&dir).unwrap();
    let mut file = File::create(&Path::new(&dir).join("mod.rs")).unwrap();

    for algebra in algebras {
        let config = codegen::read_config_from_str(&algebra);
        writeln!(file, "pub mod {};", config.algebra_name).unwrap();
        codegen::generate_code(config, &dir);
    }

    println!("cargo:rerun-if-changed=build.rs");
}
