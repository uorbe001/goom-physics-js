var requirejs = require("requirejs");
requirejs.config({nodeRequire: require});

requirejs(["goom-math", "../../src/contact", "../../src/rigid_body"], function(Mathematics, Contact, RigidBody) {
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
			//TODO: Test this more throughtly and test calculateDeltaVelocity
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
});
