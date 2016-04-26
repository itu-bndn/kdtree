module Kdtree

open Point

type Axis = X | Y | Z

[<NoComparison>]
type Bounds = Bounds of Point * w:float * h:float * d:float

[<NoComparison>]
type 'e Node =
    | Node of Axis * s:float * l:'e Node * r:'e Node
    | Leaf of 'e list

[<NoComparison>]
type 'e Tree = Tree of Bounds * 'e Node

type 'e kdtree = 'e Tree

[<Literal>]
let MaximumCost = 1.75

[<Literal>]
let MaximumOverlap = 0.6

/// <summary>
/// Get the origin of a bounding box.
/// </summary>
/// <param name=b>The bounds whose origin to get.</param>
/// <returns>The origin of the bounds.</returns>
let getOrigin = function Bounds(p, _, _, _) -> p

/// <summary>
/// Get the width of a bounding box.
/// </summary>
/// <param name=b>The bounds whose width to get.</param>
/// <returns>The width of the bounds.</returns>
let getWidth = function Bounds(_, w, _, _) -> w

/// <summary>
/// Get the height of a bounding box.
/// </summary>
/// <param name=b>The bounds whose height to get.</param>
/// <returns>The height of the bounds.</returns>
let getHeight = function Bounds(_, _, h, _) -> h

/// <summary>
/// Get the depth of a bounding box.
/// </summary>
/// <param name=b>The bounds whose depth to get.</param>
/// <returns>The depth of the bounds.</returns>
let getDepth = function Bounds(_, _, _, d) -> d

/// <summary>
/// Get the dimensions of a bounding box.
/// </summary>
/// <param name=b>The bounds whose dimensions to get.</param>
/// <returns>The dimensions of the bounds.</returns>
let getDimensions = function Bounds(_, w, h, d) -> (w, h, d)

/// <summary>
/// Get the minimum and maximum values of a bounding box.
/// </summary>
/// <param name=b>The bounds.</param>
/// <returns>The minimum and maximum values of the bounds.</returns>
let minMax b =
    let p = getOrigin b
    let x = let x' = Point.getX p in (x', x' + getWidth b)
    let y = let y' = Point.getY p in (y', y' + getHeight b)
    let z = let z' = Point.getZ p in (z', z' + getDepth b)
    (x, y, z)

/// <summary>
/// Get the minimum and maximum values of a bounding box at a given axis.
/// </summary>
/// <param name=a>The axis.</param>
/// <param name=b>The bounds.</param>
/// <returns>The minimum and maximum values of the bounds at the axis.</returns>
let minMaxAt a b =
    let (x, y, z) = minMax b
    match a with
    | X -> x
    | Y -> y
    | Z -> z

/// <summary>
/// Get the global minimum and maximum values of a list of bounding box.
/// </summary>
let globalMinMax bs =
    let f (xv, xa, yv, ya, zv, za) b =
        let ((xv', xa'), (yv', ya'), (zv', za')) = minMax b
        (
            min xv xv', max xa xa',
            min yv yv', max ya ya',
            min zv zv', max za za'
        )

    // Find the minimum and maximum values of each axis.
    let i = infinity in List.fold f (i, -i, i, -i, i, -i) bs

/// <summary>
/// Get the global bounds of a list of bounding box.
/// </summary>
let globalBounds bs =
    let (xv, xa, yv, ya, zv, za) = globalMinMax bs
    Bounds(
        Point.make xv yv zv,
        xa - xv,
        ya - yv,
        za - zv
    )

/// <summary>
/// Given a list of bounds, compute the axis with the largest span.
/// </summary>
/// <param name=bs>The list of bounds.</param>
/// <returns>The largest axis and its size.</returns>
let largestAxis b =
    let w = getWidth b
    let h = getHeight b
    let d = getDepth b

    match (max w (max h d)) with
    | n when n = w -> (X, n)
    | n when n = h -> (Y, n)
    | n            -> (Z, n)

/// <summary>
/// Given a list of bounding boxes, construct a list of candidate splits along an axis.
/// </summary>
/// <param name=a>The axis along which to find candidate splits.</param>
/// <param name=bs>The bounds for which to construct candidate splits.</param>
/// <returns>A sorted list of distinct candidate splits.</returns>
let candidateSplits a b =
    let (bv, ba) = minMaxAt a b

    let f cs b =
        // Let the minimum and maximum values be candidate splits...
        let (vv, va) = minMaxAt a b

        // ...provided that they're within the bounds we're splitting.
        let cs = if vv > bv && vv < ba then vv :: cs else cs
        let cs = if va > bv && va < ba then va :: cs else cs

        cs

    List.fold f [] >> List.distinct >> List.sort

/// <summary>
/// Compute the approximate cost of making a split along an axis.
/// </summary>
/// <remarks>
/// The cost approximation takes the following factors into account:
/// - The surface areas on both sides of the split.
/// - The number of elements on either side of the split.
/// - The number of overlaps caused by the split.
/// The higher any of these factors are, the costlier the split.
/// </remarks>
/// <param name=a>The axis to split along.</param>
/// <param name=sl>The surface area to the left of the split.</param>
/// <param name=sr>The surface area to the right of the split.
/// <param name=s>The split.</param>
/// <param name=bs>The bounds to compute the cost of splitting.</param>
/// <returns>The approximate cost of making the split.</returns>
let approximateCost a sl sr s bs =
    let f (nl, nr, nt, no) b =
        let (vv, va) = minMaxAt a b

        // Check if the bounds lie on the left and/or right side of the split.
        let nl = if vv < s then nl + 1 else nl
        let nr = if va > s then nr + 1 else nr

        // Check if the bounds overlap the split point.
        let no = if vv < s && va > s then no + 1 else no

        (nl, nr, nt + 1, no)

    let (nl, nr, nt, no) = List.fold f (0, 0, 0, 0) bs

    // Stop when too many bounds are on both sides of the split.
    if float no / float nt >= MaximumOverlap then infinity else

    (float nl / float nt) * sl + (float nr / float nt) * sr

/// <summary>
/// Compute the optimal split to make for a list of bounding boxes.
/// </summary>
/// <param name=b>The global bounds of the bounds to split.</param>
/// <param name=bs>The bounds to split.</param>
/// <returns>The value of where to make an optimal split and the axis to split along.</param>
let optimalSplit b bs =
    let (a, w) = largestAxis b
    let (l, _) = minMaxAt a b

    if w = 0.0 then None else

    let cs = candidateSplits a b bs

    let f (c, s) s' =
        // Compute the relative axis areas that the split will cause.
        let sl = (s' - l) / w
        let sr = 1. - sl

        // Compute the cost of making this split.
        let c' = approximateCost a sl sr s' bs

        // Check if we've found a better place to make the split.
        if c' < c then (c', s') else (c, s)

    match List.fold f (infinity, 0.) cs with
    | (c, s) when c < MaximumCost -> Some (a, s)
    | _ -> None

/// <summary>
/// Split a bounding box at a point along an axis.
/// </summary>
let splitBounds (a, s) b =
    let p = getOrigin b
    let x = Point.getX p
    let y = Point.getY p
    let z = Point.getZ p

    let (w, h, d) = getDimensions b

    match a with
    | X ->
        let v = s - x
        let v' = w - v
        (
            Bounds(Point.make x y z, v, h, d),
            Bounds(Point.make (x + v) y z, v', h, d)
        )
    | Y ->
        let v = s - y
        let v' = h - v
        (
            Bounds(Point.make x y z, w, v, d),
            Bounds(Point.make x (y + v) z, w, v', d)
        )
    | Z ->
        let v = s - z
        let v' = d - v
        (
            Bounds(Point.make x y z, w, h, v),
            Bounds(Point.make x y (z + v), w, h, v')
        )

/// <summary>
/// Split a list of elements at a point along an axis.
/// </summary>
let splitElements (a, s) es =
    let f (el, er) (e, b) =
        let (vv, va) = minMaxAt a b

        // Check if the bounds lie on the left and/or right side of the split.
        let el = if vv <= s then (e, b) :: el else el
        let er = if va >  s then (e, b) :: er else er

        (el, er)

    List.fold f ([], []) es

/// <summary>
/// Recursively construct a tree given a bounding box and a list of elements and their bounds.
/// </summary>
/// <remarks>
/// This function uses continuations in order to avoid unbounded stack growth.
/// </remarks>
/// <param name=b>The global bounding box for the elements.</param>
/// <param name=es>The element list to construct a tree for.</param>
/// <param name=f>The continuation function.</param>
/// <returns>The constructed tree.</returns>
let rec construct b es f =
    let bs = List.fold (fun bs (_, b) -> b :: bs) [] es

    match optimalSplit b bs with
    | Some (a, s) ->
        let (bl, br) = splitBounds (a, s) b
        let (el, er) = splitElements (a, s) es

        construct bl el (fun l ->
            construct br er (fun r ->
                f (Node(a, s, l, r))
            )
        )

    | None -> f (Leaf(List.fold (fun es (e, _) -> e :: es) [] es))

/// <summary>
/// Construct a tree from a list of elements and their bounds.
/// </summary>
/// <param name=es>The list of elements and their bounds.</param>
/// <returns>The constructed tree.</returns>
let make es =
    let f (es, bs) (e, b) =
        let b = Bounds b
        ((e, b) :: es, b :: bs)

    let (es, bs) = List.fold f ([], []) es

    let gb = globalBounds bs in Tree(gb, construct gb es id)

/// <summary>
/// Get the direction of a ray at a given axis.
/// </summary>
let direction a r =
    match a with
    | X -> Vector.getX (Ray.getVector r)
    | Y -> Vector.getY (Ray.getVector r)
    | Z -> Vector.getZ (Ray.getVector r)

/// <summary>
/// Get the origin of a ray at a given axis.
/// </summary>
let origin a r =
    match a with
    | X -> Point.getX (Ray.getOrigin r)
    | Y -> Point.getY (Ray.getOrigin r)
    | Z -> Point.getZ (Ray.getOrigin r)

/// <summary>
/// Get the distance of a ray to a bounding box at a given axis.
/// </summary>
let distance a r b =
    let d = direction a r
    let o = origin a r

    let (l, h) = minMaxAt a b

    if d >= 0.
    then ((l - o) / d, (h - o) / d)
    else ((h - o) / d, (l - o) / d)

/// <summary>
/// Traverse the elements of a tree based on intersection with a ray.
/// </summary>
/// <param name=f>The traversal function that determines if the wanted element has been found.</param>
/// <param name=r>The ray to use for traversing the tree.</param>
/// <param name=t>The tree to traverse.</param>
/// <returns>The element if found during traversal.</returns>
let traverse f r (Tree(b, n)) =
    let rec t n r tmin tmax c =
        match n with
        | Leaf [] -> c (None)
        | Leaf es -> c (f tmax es)

        | Node(a, s, ln, rn) ->
            let d = direction a r
            let o = origin a r

            // Figure out which is the near and far child.
            let (near, far) = if o < s then (ln, rn) else (rn, ln)

            match (s - o) / d with
            // If we can establish which child the ray will traverse through,
            // then we only need to traverse one of the two children.
            | thit when thit >= tmax || thit < 0. -> t near r tmin tmax c
            | thit when thit <= tmin              -> t far  r tmin tmax c
            // Otherwise, check for a hit in both children.
            | thit ->
                t near r tmin thit (fun e ->
                    match e with
                    | Some _ -> e
                    | None   -> t far r thit tmax c
                )

    let (txmin, txmax) = distance X r b
    let (tymin, tymax) = distance Y r b
    let (tzmin, tzmax) = distance Z r b

    let tmin = max txmin (max tymin tzmin)
    let tmax = min txmax (min tymax tzmax)

    if tmin < tmax && tmax > 0.
    then t n r tmin tmax id
    else None
