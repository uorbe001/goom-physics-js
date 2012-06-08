var Mathematics = require("goom-math-js"), BodyForceRegistry = require("../../src/body_force_registry"), RigidBody = require("../../src/rigid_body"), Gravity = require("../../src/Gravity");

describe('Physics.BodyForceRegistry', function() {
	beforeEach(function() {
		this.registry = new BodyForceRegistry();
	});

	it('should add a body and force generator pair to the registry', function() {
		var body = new RigidBody();
		var fg = new Gravity(new Mathematics.Vector3D(0, -10, 0));
		expect(this.registry.registrations.length).toBe(0);
		this.registry.add(body, fg);
		expect(this.registry.registrations.length).toBe(1);
		expect(this.registry.registrations[0].body).toBe(body);
		expect(this.registry.registrations[0].forceGenerator).toBe(fg);
	});

	it('should remove a body and force pair from the registry', function() {
		var body = new RigidBody();
		var fg = new Gravity(new Mathematics.Vector3D(0, -10, 0));
		this.registry.add(body, fg);
		body2 = new RigidBody();
		this.registry.add(body2, fg);
		expect(this.registry.registrations.length).toBe(2);
		this.registry.remove(body2, fg);
		expect(this.registry.registrations.length).toBe(1);
		expect(this.registry.registrations[0].body).toBe(body);
		expect(this.registry.registrations[0].forceGenerator).toBe(fg);
		this.registry.remove(body, fg);
		expect(this.registry.registrations.length).toBe(0);
	});

	it('should clear all the registrations from the registry', function() {
		var body = new RigidBody();
		var fg = new Gravity(new Mathematics.Vector3D(0, -10, 0));
		this.registry.add(body, fg);
		body2 = new RigidBody();
		this.registry.add(body2, fg);
		expect(this.registry.registrations.length).toBe(2);
		this.registry.clear();
		expect(this.registry.registrations.length).toBe(0);
	});

	it('should update all the forces in the registry', function() {
		var body = new RigidBody();
		var fg = new Gravity(new Mathematics.Vector3D(0, -10, 0));
		spyOn(fg, "updateForce").andCallThrough();
		this.registry.add(body, fg);
		var body2 = new RigidBody();
		this.registry.add(body2, fg);
		expect(this.registry.registrations.length).toBe(2);
		this.registry.updateForces(1);
		expect(fg.updateForce).toHaveBeenCalledWith(body, 1);
		expect(fg.updateForce).toHaveBeenCalledWith(body2, 1);
	});
});