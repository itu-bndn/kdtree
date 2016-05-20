module Kdtree

/// <summary>
/// Alias for the representation of an axis. Since it has three possible values
/// then we can represent it in a single byte. The overhead for a discriminated
/// union is 12 bytes per case, so a byte representation is much cheaper.
/// </summary>
type Axis = byte

/// <summary>
/// Alias for the representation of a split position.
/// </summary>
type Split = float

/// <summary>
/// Alias for the representation of bounding boxes; three coordinates and then
/// a width, a height, and a depth.
/// <summary>
type Bounds = float * float * float * float * float * float

/// <summary>
/// Type for representing elements when constructing the tree; the value of the
/// element, as specified by the caller, the bounds of the element, and two
/// mutable variables used for optimizing classification and filtering.
/// </summary>
type 'e Element =
    {
        Value: 'e;
        Bounds: Bounds;
        mutable Left: bool;
        mutable Right: bool;
    }

[<NoComparison>]
type 'e Node =
    | Node of Axis * Split * l:'e Node * r:'e Node
    | Leaf of 'e array

[<NoComparison>]
type 'e Tree = Tree of Bounds * 'e Node

/// <summary>
/// Alias for making nice-looking type definitions on the calling end.
/// </summary>
type 'e kdtree = 'e Tree

[<Literal>]
let X = 0uy

[<Literal>]
let Y = 1uy

[<Literal>]
let Z = 2uy

[<Literal>]
let S = 0uy

[<Literal>]
let E = 1uy

/// <summary>
/// The approximate cost of traversing one level down the tree. This is rather
/// expensive as every traversal step requires O(n) construction time.
/// </summary>
[<Literal>]
let TraversalCost = 100.

/// <summary>
/// The approximate cost a computing a single intersection. This is pretty
/// cheap, but in some cases making the tree one level deeper might be cheaper.
/// </summary>
[<Literal>]
let IntersectionCost = 2.

/// <summary>
/// Compute the surface area of a bounding box.
/// </summary>
let inline area (_, _, _, w, h, d) = 2. * h * w + 2. * h * d + 2. * w * d

/// <summary>
/// Generate events for a list of elements.
/// </summary>
let inline events es =
    // Generate the events for a single axis.
    let e a = es |> Array.collect (fun e ->
        let x, y, z, w, h, d = (!e).Bounds

        match a with
        | X -> [|e, x, S; e, x + w, E|]
        | Y -> [|e, y, S; e, y + h, E|]
        | _ -> [|e, z, S; e, z + d, E|]
    )

    // Generate the sorted events for all axis.
    [|X .. Z|] |> Array.map (fun a ->
        // Sort the events by their split value and type.
        e a |> Array.sortWith (fun (_, v, t) (_, u, s) ->
            if v = u then compare t s else compare v u
        )
    )

/// <summary>
/// Compute the bounds of a list of elements.
/// </summary>
let inline bounds es =
    let f (xv, xa, yv, ya, zv, za) e =
        let x, y, z, w, h, d = (!e).Bounds

        min xv x, max xa x + w,
        min yv y, max ya y + h,
        min zv z, max za z + d

    let i = infinity

    // Find the minimum and maximum values of each axis.
    let xv, xa, yv, ya, zv, za = es |> Array.fold f (i, -i, i, -i, i, -i)

    // Compute the final bounds based on the minimums and maximums.
    xv, yv, zv, xa - xv, ya - yv, za - zv

/// <summary>
/// Split a bounding box at a split position along an axis.
/// </summary>
let inline split a s (x, y, z, w, h, d) =
    match a with
    | X -> (x, y, z, s - x, h, d), (s, y, z, w - (s - x), h, d)
    | Y -> (x, y, z, w, s - y, d), (x, s, z, w, h - (s - y), d)
    | _ -> (x, y, z, w, h, s - z), (x, y, s, w, h, d - (s - z))

/// <summary>
/// Find the best plane along an axis at which to make a split.
/// </summary>
let inline plane a b ev =
    let sp = area b

    let bc = Array.fold (fun (i, i', c', s', nl, nr) (_, s, t) ->
        // Check if the element will lie on the right side of the split.
        let nr = if t = E then nr - 1 else nr

        // Split the parent bounds at the split position of the current event.
        let bl, br = split a s b

        // Compute the relative surface areas of the left and rights bounds
        // after the split has been made.
        let sl = area bl / sp
        let sr = area br / sp

        // Compute the cost of making this split.
        let c = TraversalCost + IntersectionCost * (float nl * sl + float nr * sr)

        // Check if the element will lie on the left side of the split.
        let nl = if t = S then nl + 1 else nl

        // Otherwise, check if we've encountered a better split position.
        if c < c'
        then i + 1, i,  c,  s,  nl, nr
        else i + 1, i', c', s', nl, nr
    )

    // Feed the initial values to the cost estimation.
    let bc = bc <| (0, 0, infinity, 0., 0, Array.length ev / 2)

    let _, i, c, s, _, _  = bc ev in i, c, s

/// <summary>
/// Classify events to determine whether they're left and/or right of a split.
/// <summary>
let inline classify i ev =
    let n = Array.length ev

    for i = 0 to i do
        let (e, _, t) = Array.get ev i in if t = S then (!e).Left <- true

    for i = i to n - 1 do
        let (e, _, t) = Array.get ev i in if t = E then (!e).Right <- true

/// <summary>
/// Filter events according to a classification.
/// </summary>
let inline filter es evs =
    // Split the elements into left and right parts.
    let el = es |> Array.filter (fun e -> (!e).Left)
    let er = es |> Array.filter (fun e -> (!e).Right)

    // Split the events into left and right parts.
    let evl = evs |> Array.map (fun ev ->
        ev |> Array.filter (fun (e, _, _) -> (!e).Left)
    )
    let evr = evs |> Array.map (fun ev ->
        ev |> Array.filter (fun (e, _, _) -> (!e).Right)
    )

    // Clear the classifications now that the elements have been filtered.
    do es |> Array.iter (fun e ->
        do (!e).Left  <- false
        do (!e).Right <- false
    )

    el, evl, er, evr

/// <summary>
/// Recursively construct a tree given a bounding box and a list of elements and their bounds.
/// </summary>
/// <remarks>
/// This function uses continuations in order to avoid unbounded stack growth.
/// </remarks>
let rec construct b es evs cont =
    // Start off by finding the best place to split the current bounds.
    let a, (i, c, s) =
        // Compute the best splitting plane for each axis across the events.
        evs |> Array.mapi  (fun i ev -> let a = byte i in a, plane a b ev)
            // Find the splitting plane with the lowest cost across the axis.
            |> Array.minBy (fun (_, (_, c, _)) -> c)

    // Get the events for the axis that has been chosen for the split plane.
    let ev = Array.get evs <| int a

    // If it's no longer beneficial to split the elements, i.e. the cost of
    // making the split exceeds that of intersecting all elements, then stop
    // splitting and construct a leaf.
    if c > IntersectionCost * float (Array.length ev)
    then
        Leaf(Array.map (fun e -> (!e).Value) es) |> cont
    else
        // 1st step of the split: Classify all events in order to determine on
        // which side of the split an event, and its associated element, will
        // fall. This is done by mutating the internal element representations.
        do classify i ev

        // 2nd step of the split: Filter all elements and their events in order
        // to obtain four lists; one with elements on the left, one with events
        // on the left, one with elements on the right, and finally one with
        // events on the right.
        let el, evl, er, evr = filter es evs

        // 3rd step of the split: Divide the parent bounds into two, which will
        // potentially be further subdivided.
        let bl, br = split a s b

        construct bl el evl (fun l ->
            construct br er evr (fun r ->
                Node(a, s, l, r) |> cont
            )
        )

/// <summary>
/// Construct a tree from a list of elements and their bounds.
/// </summary>
/// <param name=es>The list of elements and their bounds.</param>
/// <returns>The constructed tree.</returns>
let make es =
    // Convert the element list to an array. This gives us the benefit of much
    // faster iteration as items are then stored in contigous memory blocks
    // rather than scattered all over the place.
    // This also converts the elements to an internal, referenceable format in
    // order to allow for performant tree construction.
    let es = es |> List.toArray |> Array.map (fun (e, (p, w, h, d)) ->
        let x, y, z = Point.getCoord p in ref {
            Value = e;
            Bounds = x, y, z, w, h, d;
            Left = false;
            Right = false;
        }
    )

    // Then, compute the global bounds of the elements. These are the bounds
    // that we wish to split into several partitions. These are also stored in
    // the root of the tree and used for bootstrapping the construction of the
    // tree nodes.
    // The construction then proceeds, initially supplied a pre-processed list
    // of events to be split.
    let b = bounds es in Tree(b, construct b es (events es) id)

/// <summary>
/// Get the direction of a ray at a given axis.
/// </summary>
let inline direction a r =
    match a with
    | X -> Vector.getX (Ray.getVector r)
    | Y -> Vector.getY (Ray.getVector r)
    | _ -> Vector.getZ (Ray.getVector r)

/// <summary>
/// Get the origin of a ray at a given axis.
/// </summary>
let inline origin a r =
    match a with
    | X -> Point.getX (Ray.getOrigin r)
    | Y -> Point.getY (Ray.getOrigin r)
    | _ -> Point.getZ (Ray.getOrigin r)

/// <summary>
/// Get the distance of a ray to a bounding box at a given axis.
/// </summary>
let inline distance a r (x, y, z, w, h, d) =
    let di = direction a r
    let oi = origin a r

    let l, h = match a with
               | X -> x, x + w
               | Y -> y, y + h
               | _ -> z, z + d

    if di >= 0.
    then ((l - oi) / di, (h - oi) / di)
    else ((h - oi) / di, (l - oi) / di)

/// <summary>
/// Traverse the elements of a tree based on intersection with a ray.
/// </summary>
/// <param name=f>The traversal function that determines if the wanted element has been found.</param>
/// <param name=r>The ray to use for traversing the tree.</param>
/// <param name=t>The tree to traverse.</param>
/// <returns>The element if found during traversal.</returns>
let traverse f r (Tree(b, n)) =
    let rec t n r tmin tmax cont =
        match n with
        // If we've found an empty leaf, then there's nothing to traverse.
        | Leaf [||] -> cont (None)
        // Otherwise, if a non-empty leaf is found, then pass on the items to
        // the caller and let them determine if they've found what they're
        // looking for.
        // While the items are stored internally as arrays, we return them as
        // a list to prevent modification by the caller.
        | Leaf es -> cont (f tmax (Array.toList es))
        // Finally, if we've found a non-leaf then figure out where to go from
        // here; should we look in the left, right, or both children?
        | Node(a, s, ln, rn) ->
            let d = direction a r
            let o = origin a r

            // Figure out which is the near and far child.
            let near, far = if o < s then ln, rn else rn, ln

            match (s - o) / d with
            // If we can establish which child the ray will traverse through,
            // then we only need to traverse one of the two children.
            | thit when thit >= tmax || thit < 0. -> t near r tmin tmax cont
            | thit when thit <= tmin              -> t far  r tmin tmax cont
            // Otherwise, check for a hit in both children.
            | thit ->
                t near r tmin thit (fun e ->
                    match e with
                    | Some _ -> e |> cont
                    | None   -> t far r thit tmax cont
                )

    let txmin, txmax = distance X r b
    let tymin, tymax = distance Y r b
    let tzmin, tzmax = distance Z r b

    let tmin = max txmin (max tymin tzmin)
    let tmax = min txmax (min tymax tzmax)

    if tmin < tmax && tmax > 0. then t n r tmin tmax id else None
