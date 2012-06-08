var Mathematics = require("goom-math-js"), Primitives = require("../../src/primitives"), IntersectionTests = require("../../src/intersection_tests"), RigidBody = require("../../src/rigid_body");

describe("Physics.IntersectionTests", function() {
	beforeEach(function() {
		this.b1 = new RigidBody();
		this.off1 = new Mathematics.Matrix4D();
		this.off1.translate(0, 0, 0);
	});

	it("should detect the interection between a box and a half space", function() {
		var b1 = new Primitives.Box(this.b1, new Mathematics.Vector3D(1, 1, 1), this.off1);
		var p1 = new Primitives.Plane(null, new Mathematics.Vector3D(0, 1, 0), -0.9);
		expect(IntersectionTests.boxAndHalfSpace(b1, p1)).toBeTruthy();
	});

	it("should not detect the interection between a box and a half space when they are too far appart", function() {
		var b1 = new Primitives.Box(this.b1, new Mathematics.Vector3D(1, 1, 1), this.off1);
		var p1 = new Primitives.Plane(null, new Mathematics.Vector3D(0, 1, 0), -1.1);
		expect(IntersectionTests.boxAndHalfSpace(b1, p1)).toBeFalsy();
	});
});