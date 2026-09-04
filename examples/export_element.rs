//! Export element data for Python cross-validation

use section_properties::fea::{Tri6, gauss_points, shape_function};
use section_properties::geometry::Point;
use std::fs::File;
use std::io::Write;

fn format_array(arr: &[f64]) -> String {
    arr.iter().map(|v| format!("{:.10}", v)).collect::<Vec<_>>().join(", ")
}

fn format_matrix(mat: &[[f64; 6]; 6]) -> String {
    let mut result = String::new();
    for i in 0..6 {
        result.push_str("    [");
        for j in 0..6 {
            result.push_str(&format!("{:.10}", mat[i][j]));
            if j < 5 { result.push_str(", "); }
        }
        if i < 5 {
            result.push_str("],\n");
        } else {
            result.push_str("]\n");
        }
    }
    result
}

fn main() {
    // Create a simple channel-like element for comparison
    let points = [
        Point::new(0.0, 0.0),
        Point::new(10.0, 0.0),
        Point::new(0.0, 2.0),
        Point::new(5.0, 0.0),
        Point::new(5.0, 1.0),
        Point::new(0.0, 1.0),
    ];

    let tri6 = Tri6::from_points(
        0,
        points,
        [0, 1, 2, 3, 4, 5],
        200_000.0, // E = 200 GPa
        76_923.0,  // G
        7850.0,    // density
    ).unwrap();

    let (k_el, f_el, c_el) = tri6.torsion_properties();
    let gps = gauss_points(6);
    
    // Compute Gauss point details
    let mut gauss_details = String::new();
    for i in 0..6 {
        let sf = shape_function(&tri6.coords, (gps[i].1, gps[i].2, gps[i].3));
        let n_str = format_array(&sf.n);
        let b0_str = format_array(&sf.b[0]);
        let b1_str = format_array(&sf.b[1]);
        
gauss_details.push_str(&format!(
            "    {{\n      \"w\": {:.10},\n      \"eta\": {:.10},\n      \"xi\": {:.10},\n      \"zeta\": {:.10},\n      \"n\": [{}],\n      \"b\": [\n        [{}],\n        [{}]\n      ],\n      \"j\": {:.10},\n      \"x\": {:.10},\n      \"y\": {:.10}\n    }}{}\n",
            gps[i].0, gps[i].1, gps[i].2, gps[i].3,
            n_str, b0_str, b1_str,
            sf.j, sf.x, sf.y,
            if i < 5 { "," } else { "" }
        ));
    }
    
    // Write to JSON file
    let mut file = File::create("element_data.json").unwrap();
    writeln!(file, "{{").unwrap();
    writeln!(file, "  \"coords\": {{").unwrap();
    writeln!(file, "    \"x\": [{}],", format_array(&[
        tri6.coords[0][0], tri6.coords[0][1], tri6.coords[0][2], 
        tri6.coords[0][3], tri6.coords[0][4], tri6.coords[0][5]
    ])).unwrap();
    writeln!(file, "    \"y\": [{}]", format_array(&[
        tri6.coords[1][0], tri6.coords[1][1], tri6.coords[1][2], 
        tri6.coords[1][3], tri6.coords[1][4], tri6.coords[1][5]
    ])).unwrap();
    writeln!(file, "  }},").unwrap();
    
    writeln!(file, "  \"k_el\": [\n{}  ],", format_matrix(&k_el)).unwrap();
    writeln!(file, "  \"f_el\": [{}],", format_array(&f_el)).unwrap();
    writeln!(file, "  \"c_el\": [{}]", format_array(&c_el)).unwrap();
    writeln!(file, ",").unwrap();
    
    // Gauss points
    writeln!(file, "  \"gauss_points\": [").unwrap();
    for i in 0..6 {
        writeln!(file, "    [{:.10}, {:.10}, {:.10}, {:.10}]{}", 
            gps[i].0, gps[i].1, gps[i].2, gps[i].3, 
            if i < 5 { "," } else { "" }).unwrap();
    }
    writeln!(file, "  ],").unwrap();
    
    writeln!(file, "  \"gauss_details\": [\n{}\n  ]", gauss_details).unwrap();
    writeln!(file, "}}").unwrap();
    
    println!("Element data exported to element_data.json");
}