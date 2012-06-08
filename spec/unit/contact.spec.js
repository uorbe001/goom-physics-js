var Mathematics = require("goom-math-js"), Contact = require("../../src/contact"), RigidBody = require("../../src/rigid_body");

describe("Physics.Contact", function() {
	beforeEach(function() {
		this.contact = new Contact(new Mathematics.Vector3D(1, 2, 3), new Mathematics.Vector3D(0, 1, 0), 1.2);
		this.b0 = new RigidBody();
		this.b1 = new RigidBody();
		this.contact.setContactData(this.b0, this.b1, 0.3, 0.4);
	});

	it("should hold a contact point, normal, and penetration, empty and 0 by default", function() {
		var contact = new Contact();
		expect(contact.point.x).toBe(0);
		expect(contact.point.y).toBe(0);
		expect(contact.point.z).toBe(0);
		expect(contact.normal.x).toBe(0);
		expect(contact.normal.y).toBe(0);
		expect(contact.normal.z).toBe(0);
		expect(contact.penetration).toBe(0);
	});

	it("should hold a contact point, normal, and penetration with the given values", function() {
		var v1 = new Mathematics.Vector3D(1, 2, 3);
		var v2 = new Mathematics.Vector3D(3, 2, 1);
		var contact = new Contact(v1, v2, 1.2);
		expect(contact.point).not.toBe(v1);
		expect(contact.point.x).toBe(1);
		expect(contact.point.y).toBe(2);
		expect(contact.point.z).toBe(3);
		expect(contact.normal).not.toBe(v2);
		expect(contact.normal.x).toBe(3);
		expect(contact.normal.y).toBe(2);
		expect(contact.normal.z).toBe(1);
		expect(contact.penetration).toBe(1.2);
	});

	it("should be able to calculate internal data from the state data", function() {
		this.contact.calculateInternalData(0.02);
		expect(this.contact.toWorld.data[0]).toBe(0);
		expect(this.contact.toWorld.data[1]).toBe(1);
		expect(this.contact.toWorld.data[2]).toBe(0);
		expect(this.contact.toWorld.data[3]).toBe(0);
		expect(this.contact.toWorld.data[4]).toBe(0);
		expect(this.contact.toWorld.data[5]).toBe(1);
		expect(this.contact.toWorld.data[6]).toBe(1);
		expect(this.contact.toWorld.data[7]).toBe(0);
		expect(this.contact.toWorld.data[8]).toBe(0);
		expect(this.contact.velocity.x).toBe(0);
		expect(this.contact.velocity.y).toBe(0);
		expect(this.contact.velocity.z).toBe(0);
		expect(this.contact.desiredDeltaVelocity).toBe(0);
		this.contact.normal.y = 0;
		this.contact.normal.x = 1;
		this.contact.normal.z = 0;
		this.contact.calculateInternalData(0.02);
		expect(this.contact.toWorld.data[0]).toBe(1);
		expect(this.contact.toWorld.data[1]).toBe(0);
		expect(this.contact.toWorld.data[2]).toBe(0);
		expect(this.contact.toWorld.data[3]).toBe(0);
		expect(this.contact.toWorld.data[4]).toBe(0);
		expect(this.contact.toWorld.data[5]).toBe(-1);
		expect(this.contact.toWorld.data[6]).toBe(0);
		expect(this.contact.toWorld.data[7]).toBe(1);
		expect(this.contact.toWorld.data[8]).toBe(0);
		expect(this.contact.velocity.x).toBe(0);
		expect(this.contact.velocity.y).toBe(0);
		expect(this.contact.velocity.z).toBe(0);
		expect(this.contact.desiredDeltaVelocity).toBe(0);

		this.b0.position.set(0, 0, 0);
		this.b0.velocity.set(0, -1, 0);
		this.b0.angular_velocity.set(0.1, 0, -0.1);
		this.b1.position.set(0, -1, 0);
		this.contact.normal.set(0, 1, 0);
		this.contact.penetration = 0.1;
		this.contact.point.set(1, -1, 1);
		this.contact.restitution = 0;

		this.contact.calculateInternalData(0.02);
		expect(this.contact.velocity.x).toBe(-1.2);
		expect(this.contact.velocity.y).toBe(-0.1);
		expect(this.contact.velocity.z).toBe(-0.1);
		expect(this.contact.desiredDeltaVelocity).toBe(1.2);
	});

	it("should resolve the position of an object in the correct direction", function() {
		var linear_change = [], angular_change = [];
		linear_change[0] = new Mathematics.Vector3D(), linear_change[1] = new Mathematics.Vector3D();
		angular_change[0] = new Mathematics.Vector3D(), angular_change[1] = new Mathematics.Vector3D();
		this.b0.setInertiaTensorCoefficients(1/12, 1/12, 1/12);
		this.b0.setMass(1);
		this.b1.setInertiaTensorCoefficients(1/12, 1/12, 1/12);
		this.b1.setMass(1);
		this.b0.position.set(0, 0, 0);
		this.b0.velocity.set(0, -1, 0);
		this.b1.position.set(0, -1, 0);
		this.contact.normal.set(0, 1, 0);
		this.contact.penetration = 0.1;
		this.contact.point.set(1, -0.5, 1);
		this.contact.restitution = 0;
		this.b0.calculateInternalData(); this.b1.calculateInternalData();
		this.contact.calculateInternalData(0.02);
		this.contact.resolvePosition(linear_change, angular_change, 0.1);

		expect(linear_change[0].x).toBe(0);
		expect(linear_change[0].y).toBeGreaterThan(0);
		expect(linear_change[0].z).toBe(0);
		expect(linear_change[1].x).toBe(0);
		expect(linear_change[1].y).toBeLessThan(0);
		expect(linear_change[1].z).toBe(0);
		expect(this.b0.position.x).toBe(0);
		expect(this.b0.position.y).toBeGreaterThan(0);
		expect(this.b0.position.z).toBe(0);
		expect(this.b1.position.x).toBe(0);
		expect(this.b1.position.y).toBeLessThan(-1);
		expect(this.b1.position.z).toBe(0);
		expect(angular_change[0].x).toBeGreaterThan(0);
		expect(angular_change[0].y).toBe(0);
		expect(angular_change[0].z).toBe(0);
		expect(angular_change[1].x).toBeLessThan(0);
		expect(angular_change[1].y).toBe(0);
		expect(angular_change[1].z).toBe(0);
	});

	it("should resolve the velocity of the objects in the correct direction", function() {
		var linear_change = [], angular_change = [];
		linear_change[0] = new Mathematics.Vector3D(), linear_change[1] = new Mathematics.Vector3D();
		angular_change[0] = new Mathematics.Vector3D(), angular_change[1] = new Mathematics.Vector3D();
		this.b0.setInertiaTensorCoefficients(1/12, 1/12, 1/12);
		this.b0.setMass(1);
		this.b1.setInertiaTensorCoefficients(1/12, 1/12, 1/12);
		this.b1.setMass(1);
		this.b0.position.set(0, 0, 0);
		this.b0.velocity.set(0, -1, 0);
		this.b1.position.set(0, -1, 0);
		this.contact.normal.set(0, 1, 0);
		this.contact.penetration = 0.1;
		this.contact.point.set(1, -1, 1);
		this.contact.restitution = 0;
		this.contact.friction = 0;
		this.b0.calculateInternalData(); this.b1.calculateInternalData();
		this.contact.calculateInternalData(0.02);
		this.contact.resolveVelocity(linear_change, angular_change);
		expect(linear_change[0].x).toBe(0);
		expect(linear_change[0].y).toBeGreaterThan(0);
		expect(linear_change[0].z).toBe(0);
		expect(linear_change[1].x).toBe(0);
		expect(linear_change[1].y).toBeLessThan(0);
		expect(linear_change[1].z).toBe(0);
		expect(this.b0.velocity.x).toBe(0);
		expect(this.b0.velocity.y).toBeGreaterThan(-1);
		expect(this.b0.velocity.z).toBe(0);
		expect(this.b1.velocity.x).toBe(0);
		expect(this.b1.velocity.y).toBeLessThan(0);
		expect(this.b1.velocity.z).toBe(0);
		expect(angular_change[0].x).toBeGreaterThan(0);
		expect(angular_change[0].y).toBe(0);
		expect(angular_change[0].z).toBe(0);
		expect(angular_change[1].x).toBeLessThan(0);
		expect(angular_change[1].y).toBe(0);
		expect(angular_change[1].z).toBe(0);
		expect(this.b0.angular_velocity.x).toBe(0.1 + angular_change[0].x);
		expect(this.b0.angular_velocity.y).toBe(0);
		expect(this.b0.angular_velocity.z).toBe(-0.1 + angular_change[0].y);
		expect(this.b1.angular_velocity.x).toBe(angular_change[1].x);
		expect(this.b1.angular_velocity.y).toBe(0);
		expect(this.b1.angular_velocity.z).toBe(angular_change[1].y);
	});

	it("should swap the bodies in the contact and update the contact normal", function() {
		this.contact.__swapBodies();
		expect(this.contact.bodies[0]).toBe(this.b1);
		expect(this.contact.bodies[1]).toBe(this.b0);
		expect(this.contact.normal.x).toBe(0);
		expect(this.contact.normal.y).toBe(-1);
		expect(this.contact.normal.z).toBe(0);
	});
});