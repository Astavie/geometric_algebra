mod algebra;
mod ast;
mod compile;
mod emit;
mod glsl;
mod rust;

use crate::{
    algebra::{BasisElement, GeometricAlgebra, Involution, MultiVectorClass, MultiVectorClassRegistry, Product},
    ast::{AstNode, DataType, Parameter},
    emit::Emitter,
};

// TODO see also the already existing "GeometricAlgebra" that is just the generator squares
pub struct AlgebraDescriptor {
    pub algebra_name: String,
    pub generator_squares: Vec<isize>,
    pub multi_vectors: Vec<MultiVectorClass>,
}

pub fn read_config_from_env() -> AlgebraDescriptor {
    let mut args = std::env::args();
    let _executable = args.next().unwrap();
    let config = args.next().unwrap();
    read_config_from_str(config.as_str())
}

pub fn read_config_from_str(config: &str) -> AlgebraDescriptor {
    let config = config.to_string();
    let mut config_iter = config.split(';');
    let algebra_descriptor = config_iter.next().unwrap();
    let mut algebra_descriptor_iter = algebra_descriptor.split(':');
    let algebra_name = algebra_descriptor_iter.next().unwrap();
    let generator_squares = algebra_descriptor_iter
        .next()
        .unwrap()
        .split(',')
        .map(|x| x.parse::<isize>().unwrap())
        .collect::<Vec<_>>();
    let algebra = GeometricAlgebra {
        generator_squares: generator_squares.as_slice(),
    };

    let mut multi_vectors = vec![];
    for multi_vector_descriptor in config_iter {
        let mut multi_vector_descriptor_iter = multi_vector_descriptor.split(':');
        multi_vectors.push(MultiVectorClass {
            class_name: multi_vector_descriptor_iter.next().unwrap().to_owned(),
            grouped_basis: multi_vector_descriptor_iter
                .next()
                .unwrap()
                .split('|')
                .map(|group_descriptor| {
                    group_descriptor
                        .split(',')
                        .map(|element_name| BasisElement::parse(element_name, &algebra))
                        .collect::<Vec<_>>()
                })
                .collect::<Vec<_>>(),
        });
    }

    AlgebraDescriptor {
        algebra_name: algebra_name.to_string(),
        generator_squares,
        multi_vectors,
    }
}

pub fn generate_code(desc: AlgebraDescriptor, path: &str) {
    // generate code
    let algebra = GeometricAlgebra {
        generator_squares: desc.generator_squares.as_slice(),
    };
    let involutions = Involution::involutions(&algebra);
    let products = Product::products(&algebra);
    let basis = algebra.sorted_basis();
    for b in basis.iter() {
        for a in basis.iter() {
            print!("{:1$} ", BasisElement::product(a, b, &algebra), desc.generator_squares.len() + 2);
        }
        println!();
    }
    let mut registry = MultiVectorClassRegistry::default();
    for multi_vector_class in desc.multi_vectors {
        registry.register(multi_vector_class);
    }
    let mut emitter = Emitter::new(&std::path::Path::new(path).join(desc.algebra_name.as_str()));
    emitter.emit(&AstNode::Preamble).unwrap();
    for class in registry.classes.iter() {
        emitter.emit(&AstNode::ClassDefinition { class }).unwrap();
    }
    let mut trait_implementations = std::collections::BTreeMap::new();
    for class_a in registry.classes.iter() {
        let parameter_a = Parameter {
            name: "self",
            data_type: DataType::MultiVector(class_a),
        };
        let mut single_trait_implementations = std::collections::BTreeMap::new();
        for (name, method) in [("Zero", "zero"), ("One", "one")] {
            let ast_node = class_a.constant(name, method);
            emitter.emit(&ast_node).unwrap();
            if ast_node != AstNode::None {
                single_trait_implementations.insert(name.to_string(), ast_node);
            }
        }
        for (name, method, involution) in involutions.iter() {
            let ast_node = MultiVectorClass::involution(name, method, involution, &parameter_a, &registry, false);
            emitter.emit(&ast_node).unwrap();
            if ast_node != AstNode::None {
                single_trait_implementations.insert(name.to_string(), ast_node);
            }
        }
        let mut pair_trait_implementations = std::collections::BTreeMap::new();
        for class_b in registry.classes.iter() {
            let mut trait_implementations = std::collections::BTreeMap::new();
            let parameter_b = Parameter {
                name: "other",
                data_type: DataType::MultiVector(class_b),
            };
            if class_a != class_b {
                let name = "Into";
                let ast_node = MultiVectorClass::involution(name, "into", &Involution::projection(class_b), &parameter_a, &registry, true);
                emitter.emit(&ast_node).unwrap();
                if ast_node != AstNode::None {
                    trait_implementations.insert(name.to_string(), ast_node);
                }
            }
            for (name, method) in [("Add", "add"), ("Sub", "sub")] {
                let ast_node = MultiVectorClass::element_wise(name, method, &parameter_a, &parameter_b, &registry);
                emitter.emit(&ast_node).unwrap();
                if ast_node != AstNode::None {
                    trait_implementations.insert(name.to_string(), ast_node);
                }
            }
            for (name, alias, product) in products.iter() {
                let ast_node = MultiVectorClass::product(name.0, name.1, product, &parameter_a, &parameter_b, &registry);
                emitter.emit(&ast_node).unwrap();

                if ast_node != AstNode::None {
                    if let Some(alias) = alias {
                        let alias_node = MultiVectorClass::alias_trait(alias.0, alias.1, &ast_node);
                        emitter.emit(&alias_node).unwrap();
                    }

                    trait_implementations.insert(name.0.to_string(), ast_node);
                }
            }
            pair_trait_implementations.insert(
                parameter_b.multi_vector_class().class_name.clone(),
                (parameter_b.clone(), trait_implementations),
            );
        }
        for (parameter_b, pair_trait_implementations) in pair_trait_implementations.values() {
            if let Some(scalar_product) = pair_trait_implementations.get("ScalarProduct") {
                if let Some(reversal) = single_trait_implementations.get("Reverse") {
                    if parameter_a.multi_vector_class() == parameter_b.multi_vector_class() {
                        let squared_magnitude = MultiVectorClass::derive_squared_magnitude(scalar_product, reversal, &parameter_a);
                        emitter.emit(&squared_magnitude).unwrap();
                        let magnitude = MultiVectorClass::derive_magnitude(&squared_magnitude, &parameter_a);
                        emitter.emit(&magnitude).unwrap();
                        single_trait_implementations.insert("SquaredNorm".to_string(), squared_magnitude);
                        single_trait_implementations.insert("Norm".to_string(), magnitude);
                    }
                }
            }
        }
        for (parameter_b, pair_trait_implementations) in pair_trait_implementations.values() {
            if let Some(geometric_product) = pair_trait_implementations.get("GeometricProduct") {
                if parameter_b.data_type.is_scalar() {
                    if let Some(magnitude) = single_trait_implementations.get("Norm") {
                        let signum = MultiVectorClass::derive_signum(geometric_product, magnitude, &parameter_a);
                        emitter.emit(&signum).unwrap();
                        single_trait_implementations.insert("Normalization".to_string(), signum);
                    }
                    if let Some(squared_magnitude) = single_trait_implementations.get("SquaredNorm") {
                        if let Some(reversal) = single_trait_implementations.get("Reverse") {
                            let inverse = MultiVectorClass::derive_inverse(geometric_product, squared_magnitude, reversal, &parameter_a);
                            emitter.emit(&inverse).unwrap();
                            single_trait_implementations.insert("Inverse".to_string(), inverse);
                        }
                    }
                }
            }
        }
        trait_implementations.insert(
            parameter_a.multi_vector_class().class_name.clone(),
            (parameter_a.clone(), single_trait_implementations, pair_trait_implementations),
        );
    }
    for (parameter_a, single_trait_implementations, pair_trait_implementations) in trait_implementations.values() {
        for (parameter_b, pair_trait_implementations) in pair_trait_implementations.values() {
            if let Some(geometric_product) = pair_trait_implementations.get("GeometricProduct") {
                let geometric_product_result = result_of_trait!(geometric_product);
                if let Some(b_trait_implementations) = trait_implementations.get(&parameter_b.multi_vector_class().class_name) {
                    if let Some(inverse) = b_trait_implementations.1.get("Inverse") {
                        let division = MultiVectorClass::derive_division(geometric_product, inverse, parameter_a, parameter_b);
                        emitter.emit(&division).unwrap();
                        let alias = MultiVectorClass::alias_trait("Div", "div", &division);
                        emitter.emit(&alias).unwrap();
                    }
                }
                if let Some(reversal) = single_trait_implementations.get("Reverse") {
                    if let Some(b_trait_implementations) = trait_implementations.get(&geometric_product_result.multi_vector_class().class_name) {
                        if let Some(b_pair_trait_implementations) = b_trait_implementations.2.get(&parameter_a.multi_vector_class().class_name) {
                            if let Some(geometric_product_2) = b_pair_trait_implementations.1.get("GeometricProduct") {
                                let geometric_product_2_result = result_of_trait!(geometric_product_2);
                                if let Some(c_trait_implementations) =
                                    trait_implementations.get(&geometric_product_2_result.multi_vector_class().class_name)
                                {
                                    if let Some(c_pair_trait_implementations) =
                                        c_trait_implementations.2.get(&parameter_b.multi_vector_class().class_name)
                                    {
                                        let transformation = MultiVectorClass::derive_sandwich_product(
                                            geometric_product,
                                            geometric_product_2,
                                            reversal,
                                            c_pair_trait_implementations.1.get("Into"),
                                            parameter_a,
                                            parameter_b,
                                        );
                                        emitter.emit(&transformation).unwrap();
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
