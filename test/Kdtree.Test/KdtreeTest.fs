/// Copyright (C) 2016 The Authors.
module Kdtree.Test

open Xunit
open FsUnit.Xunit

let e = [
    ("foo", (Point.make 0. 0. -1., 2., 2., 2.));
    ("bar", (Point.make 4. 4. -1., 2., 2., 2.));
]

let t = make e

[<Fact>]
let ``make constructs a kd-tree given a list of elements and their bounds`` () =
    // Check that a kd-tree of strings is constructed.
    make e |> should be instanceOfType<string kdtree>

[<Fact>]
let ``traverse traverses the elements of a kd-tree based on intersection with a ray`` () =
    // Ray shot from below "foo" directly at it.
    let r1 = Ray.make (Point.make 0.5 -0.5 0.) (Vector.make 0. 1. 0.)

    traverse (fun d e -> Some e) r1 t
    |> should equal (Some ["foo"; "bar"])

    // Let's check if traversal stops if we find nothing.
    traverse (fun d e -> None) r1 t
    |> should equal None

    // Now, let's ignore "foo" and check that nothing else is hit.
    traverse (fun d e -> if e = ["foo"; "bar"] then None else Some e) r1 t
    |> should equal None

    // Let's instead shoot out a ray that goes through the side of "foo" and later hits "bar".
    let r2 = Ray.make (Point.make 0.5 -0.5 0.) (Vector.make 3. 4. 0.)

    // Once again, ignore hits on "foo" and check that we instead hit "bar".
    traverse (fun d e -> if e = ["foo"; "bar"] then Some ["bar"] else Some e) r2 t
    |> should equal (Some ["bar"])
