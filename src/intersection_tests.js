var Mathematics = require("goom-math");

/**
	@namespace Holds interesection tests used for early-out checks.
*/
var IntersectionTests = {};
IntersectionTests.__helperVector = new Mathematics.Vector3D();

/**
	Checks whether the given primitives intersect or not.
	@param {Physics.Box} box The box to check.
	@param {Physics.Plane} plane The plane to check.
	@returns {Boolean} true if they intersect, false otherwise.
*/
IntersectionTests.boxAndHalfSpace = function(box, plane) {
	var box_halfsize = box.halfSize;
	var projected_radious = box_halfsize.x * Math.abs(plane.normal.dotProduct(box.axisVector(0, IntersectionTests.__helperVector))) + box_halfsize.y * Math.abs(plane.normal.dotProduct(box.axisVector(1, IntersectionTests.__helperVector))) + box_halfsize.z * Math.abs(plane.normal.dotProduct(box.axisVector(2, IntersectionTests.__helperVector)));
	var box_distance = plane.normal.dotProduct(box.position(IntersectionTests.__helperVector)) - projected_radious;
	return box_distance <= plane.offset;
};

module.exports = IntersectionTests;