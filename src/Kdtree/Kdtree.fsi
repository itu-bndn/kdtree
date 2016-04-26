module Kdtree

open Point
open Ray

[<NoComparison>]
type 'e Tree

type 'e kdtree = 'e Tree

/// <summary>
/// Construct a tree from a list of elements and their bounds.
/// </summary>
/// <param name=es>The list of elements and their bounds.</param>
/// <returns>The constructed tree.</returns>
val make : es:('e * (Point * float * float * float)) list -> 'e Tree

/// <summary>
/// Traverse the elements of a tree based on intersection with a ray.
/// </summary>
/// <param name=f>The traversal function that determines if the wanted element has been found.</param>
/// <param name=r>The ray to use for traversing the tree.</param>
/// <param name=t>The tree to traverse.</param>
/// <returns>The element if found during traversal.</returns>
val traverse : f:(float -> 'e list -> 'a option) -> r:Ray -> t:'e Tree -> 'a option
