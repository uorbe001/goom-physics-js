var Gravity = require("../../src/gravity"), RigidBody = require("../../src/rigid_body"), Mathematics = require("goom-math");

describe('Gravity', function() {
	beforeEach(function() {
		var force = new Mathematics.Vector3D(0, -10, 0);
		this.gravity = new Gravity(force);
	});

	it("should call the body's apply force method with the gravity force", function() {
		var body = new RigidBody();
		body.setMass(1);
		spyOn(body, "applyForce");
		this.gravity.updateForce(body, 1);
		expect(body.applyForce).toHaveBeenCalledWith(new Mathematics.Vector3D(0, -10, 0));

		body.setMass(10);
		this.gravity.updateForce(body, 10);
		expect(body.applyForce).toHaveBeenCalledWith(new Mathematics.Vector3D(0*10, -10*10, 0*10));
		expect(this.gravity.gravity.x).toBe(0);
		expect(this.gravity.gravity.y).toBe(-10);
		expect(this.gravity.gravity.z).toBe(0);
	});
});