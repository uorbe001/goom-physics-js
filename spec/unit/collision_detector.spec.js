var Mathematics = require("goom-math-js"), Contact = require("../../src/contact"), IntersectionTests = require("../../src/intersection_tests"),
	Primitives = require("../../src/primitives"), RigidBody = require("../../src/rigid_body"), CollisionDetector = require("../../src/collision_detector");

describe("Physics.CollisionDetector", function() {
	beforeEach(function() {
		this.cd = new CollisionDetector();
		this.contacts = [];
		this.b1 = new RigidBody();
		this.b2 = new RigidBody();
		this.data = {restitution: 0.6, friction: 0.4};
	});

	it("should return the last contact in the cache", function() {
		expect(this.cd.contactCacheTop).toBe(0);
		var c = this.cd.__getFreeContact();
		expect(this.cd.contactCacheTop).toBe(1);
		expect(c !== undefined).toBeTruthy();
	});

	it("should return a new contact when the contact cache is full", function() {
		var c;

		for (var i = 1; i <= this.cd.MAX_CONTACTS; i++) {
			c = this.cd.__getFreeContact();
			expect(this.cd.contactCacheTop).toBe(i);
			expect(this.cd.contactCache[i - 1]).toBe(c);
		}

		//The cache is full now.
		c = this.cd.__getFreeContact();
		expect(this.cd.contactCacheTop).toBe(this.cd.MAX_CONTACTS);
		for (i = 1; i <= this.cd.MAX_CONTACTS; i++) {
			expect(this.cd.contactCache[i - 1]).not.toBe(c);
		}
	});

	it("should detect a collision between two spheres", function() {
		var off1 = new Mathematics.Matrix4D();
		off1.translate(-1, 0, 0);
		var off2 = new Mathematics.Matrix4D();
		off2.translate(0.9, 0, 0);
		var s1 = new Primitives.Sphere(this.b1, 1, off1);
		var s2 = new Primitives.Sphere(this.b2, 1, off2);
		expect(this.contacts.length).toBe(0);
		expect(this.cd.sphereAndSphere(s1, s2, this.data, this.contacts)).toBeTruthy();
		expect(this.contacts.length).toBe(1);
		expect(this.contacts[0].normal.x).toBe(-1);
		expect(this.contacts[0].normal.y).toBe(0);
		expect(this.contacts[0].normal.z).toBe(0);
		expect(Math.round(this.contacts[0].point.x)).toBe(0);
		expect(Math.round(this.contacts[0].point.y*10)/10).toBe(0);
		expect(Math.round(this.contacts[0].point.z*10)/10).toBe(0);
		expect(Math.round(this.contacts[0].penetration*10)/10).toBe(0.1);
		expect(this.contacts[0].restitution).toBe(0.6);
		expect(this.contacts[0].friction).toBe(0.4);
		expect(this.contacts[0].bodies[0]).toBe(this.b1);
		expect(this.contacts[0].bodies[1]).toBe(this.b2);


		off1 = new Mathematics.Matrix4D();
		off1.translate(0, 3, 0);
		off2 = new Mathematics.Matrix4D();
		off2.translate(0, 1.1, 0);
		s1 = new Primitives.Sphere(this.b1, 1, off1);
		s2 = new Primitives.Sphere(this.b2, 1, off2);
		expect(this.cd.sphereAndSphere(s1, s2, this.data, this.contacts)).toBeTruthy();
		expect(this.contacts.length).toBe(2);
		expect(this.contacts[1].normal.x).toBe(0);
		expect(this.contacts[1].normal.y).toBe(1);
		expect(this.contacts[1].normal.z).toBe(0);
		expect(this.contacts[1].point.x).toBe(0);
		expect(Math.round(this.contacts[1].point.y)).toBe(2);
		expect(this.contacts[1].point.z).toBe(0);
		expect(Math.round(this.contacts[1].penetration * 100)/100).toBe(0.1);
		expect(this.contacts[1].restitution).toBe(0.6);
		expect(this.contacts[1].friction).toBe(0.4);
		expect(this.contacts[1].bodies[0]).toBe(this.b1);
		expect(this.contacts[1].bodies[1]).toBe(this.b2);

		expect(this.cd.sphereAndSphere(s2, s1, this.data, this.contacts)).toBeTruthy();
		expect(this.contacts.length).toBe(3);
		expect(this.contacts[2].normal.x).toBe(0);
		expect(this.contacts[2].normal.y).toBe(-1);
		expect(this.contacts[2].normal.z).toBe(0);
		expect(this.contacts[2].point.x).toBe(0);
		expect(Math.round(this.contacts[2].point.y)).toBe(2);
		expect(this.contacts[2].point.z).toBe(0);
		expect(Math.round(this.contacts[2].penetration * 100)/100).toBe(0.1);
		expect(this.contacts[2].restitution).toBe(0.6);
		expect(this.contacts[2].friction).toBe(0.4);
		expect(this.contacts[2].bodies[0]).toBe(this.b2);
		expect(this.contacts[2].bodies[1]).toBe(this.b1);

		off1.makeTranslation(-1, 0, 0);
		off2.makeTranslation(0.9, 0, 0);
		s1 = new Primitives.Sphere(this.b1, 1, off1);
		s2 = new Primitives.Sphere(this.b2, 1, off2);
		expect(this.cd.sphereAndSphere(s2, s1, this.data, this.contacts)).toBeTruthy();
		expect(this.contacts.length).toBe(4);
		expect(this.contacts[3].normal.x).toBe(1);
		expect(this.contacts[3].normal.y).toBe(0);
		expect(this.contacts[3].normal.z).toBe(0);
		expect(Math.round(this.contacts[3].point.x)).toBe(0);
		expect(Math.round(this.contacts[3].point.y*10)/10).toBe(0);
		expect(Math.round(this.contacts[3].point.z*10)/10).toBe(0);
		expect(Math.round(this.contacts[3].penetration*10)/10).toBe(0.1);
		expect(this.contacts[3].restitution).toBe(0.6);
		expect(this.contacts[3].friction).toBe(0.4);
		expect(this.contacts[3].bodies[0]).toBe(this.b2);
		expect(this.contacts[3].bodies[1]).toBe(this.b1);
	});

	it("should not detect any collision with two spheres too far appart", function() {
		var off1 = new Mathematics.Matrix4D();
		off1.makeTranslation(-1, 0, 0);
		var off2 = new Mathematics.Matrix4D();
		off2.makeTranslation(1, 0, 0);
		var s1 = new Primitives.Sphere(this.b1, 1, off1);
		var s2 = new Primitives.Sphere(this.b2, 1, off2);
		expect(this.cd.sphereAndSphere(s1, s2, this.data, this.contacts)).toBeFalsy();
		expect(this.contacts.length).toBe(0);
		expect(this.cd.sphereAndSphere(s2, s1, this.data, this.contacts)).toBeFalsy();
		expect(this.contacts.length).toBe(0);

		off1.makeTranslation(0, -1, 0);
		off2.makeTranslation(0, 1, 0);
		s1 = new Primitives.Sphere(this.b1, 1, off1);
		s2 = new Primitives.Sphere(this.b2, 1, off2);
		expect(this.cd.sphereAndSphere(s1, s2, this.data, this.contacts)).toBeFalsy();
		expect(this.contacts.length).toBe(0);
		expect(this.cd.sphereAndSphere(s2, s1, this.data, this.contacts)).toBeFalsy();
		expect(this.contacts.length).toBe(0);

		off1.makeTranslation(0, 0, -1);
		off2.makeTranslation(0, 0, 1);
		s1 = new Primitives.Sphere(this.b1, 1, off1);
		s2 = new Primitives.Sphere(this.b2, 1, off2);
		expect(this.cd.sphereAndSphere(s1, s2, this.data, this.contacts)).toBeFalsy();
		expect(this.contacts.length).toBe(0);
		expect(this.cd.sphereAndSphere(s2, s1, this.data, this.contacts)).toBeFalsy();
		expect(this.contacts.length).toBe(0);
	});

	it("should detect a collision between a sphere and a half-space", function() {
		var off1 = new Mathematics.Matrix4D();
		off1.makeTranslation(0, 0, 0);
		var s1 = new Primitives.Sphere(this.b1, 1, off1);
		var p1 = new Primitives.Plane(null, new Mathematics.Vector3D(0, 1, 0), -0.9);
		expect(this.contacts.length).toBe(0);
		expect(this.cd.sphereAndHalfSpace(s1, p1, this.data, this.contacts)).toBeTruthy();
		expect(this.contacts.length).toBe(1);
		expect(this.contacts[0].normal.x).toBe(0);
		expect(this.contacts[0].normal.y).toBe(1);
		expect(this.contacts[0].normal.z).toBe(0);
		expect(this.contacts[0].point.x).toBe(0);
		expect(this.contacts[0].point.y).toBe(-0.9);
		expect(this.contacts[0].point.z).toBe(0);
		expect(Math.round(this.contacts[0].penetration*100)/100).toBe(0.1);
		expect(this.contacts[0].restitution).toBe(0.6);
		expect(this.contacts[0].friction).toBe(0.4);
		expect(this.contacts[0].bodies[0]).toBe(this.b1);
		expect(this.contacts[0].bodies[1]).toBe(null);

		off1.makeTranslation(0.9, 0, 0);
		s1 = new Primitives.Sphere(this.b1, 1, off1);
		p1 = new Primitives.Plane(null, new Mathematics.Vector3D(1, 0, 0), 0);
		expect(this.cd.sphereAndHalfSpace(s1, p1, this.data, this.contacts)).toBeTruthy();
		expect(this.contacts.length).toBe(2);
		expect(this.contacts[1].normal.x).toBe(1);
		expect(this.contacts[1].normal.y).toBe(0);
		expect(this.contacts[1].normal.z).toBe(0);
		expect(this.contacts[1].point.x).toBe(0);
		expect(this.contacts[1].point.y).toBe(0);
		expect(this.contacts[1].point.z).toBe(0);
		expect(Math.round(this.contacts[1].penetration*100)/100).toBe(0.1);
		expect(this.contacts[1].restitution).toBe(0.6);
		expect(this.contacts[1].friction).toBe(0.4);
		expect(this.contacts[1].bodies[0]).toBe(this.b1);
		expect(this.contacts[1].bodies[1]).toBe(null);
	});

	it("should not detect a collision between a sphere and a half-space when they are too far appart", function() {
		var off1 = new Mathematics.Matrix4D();
		off1.makeTranslation(0, 0, 0);
		var s1 = new Primitives.Sphere(this.b1, 1, off1);
		var p1 = new Primitives.Plane(null, new Mathematics.Vector3D(0, 1, 0), -1);
		expect(this.contacts.length).toBe(0);
		expect(this.cd.sphereAndHalfSpace(s1, p1, this.data, this.contacts)).toBeFalsy();
		expect(this.contacts.length).toBe(0);

		off1.makeTranslation(1, 0, 0);
		s1 = new Primitives.Sphere(this.b1, 1, off1);
		p1 = new Primitives.Plane(null, new Mathematics.Vector3D(1, 0, 0), 0);
		expect(this.cd.sphereAndHalfSpace(s1, p1, this.data, this.contacts)).toBeFalsy();
		expect(this.contacts.length).toBe(0);
	});


	it("should detect a collision between a box and a half-space", function() {
		var off1 = new Mathematics.Matrix4D();
		off1.makeTranslation(0, 0, 0);
		var b1 = new Primitives.Box(this.b1, new Mathematics.Vector3D(1, 1, 1), off1);
		var p1 = new Primitives.Plane(null, new Mathematics.Vector3D(0, 1, 0), -0.9);
		expect(this.cd.boxAndHalfSpace(b1, p1, this.data, this.contacts)).toBeTruthy();
		expect(this.contacts.length).toBe(4);
		expect(this.contacts[0].normal.x).toBe(0);
		expect(this.contacts[0].normal.y).toBe(1);
		expect(this.contacts[0].normal.z).toBe(0);
		expect(this.contacts[0].point.x).toBe(1);
		expect(this.contacts[0].point.y).toBe(-0.9);
		expect(this.contacts[0].point.z).toBe(1);
		expect(Math.round(this.contacts[0].penetration*100)/100).toBe(0.1);
		expect(this.contacts[0].restitution).toBe(0.6);
		expect(this.contacts[0].friction).toBe(0.4);
		expect(this.contacts[0].bodies[0]).toBe(this.b1);
		expect(this.contacts[0].bodies[1]).toBe(null);
		expect(this.contacts[1].normal.x).toBe(0);
		expect(this.contacts[1].normal.y).toBe(1);
		expect(this.contacts[1].normal.z).toBe(0);
		expect(this.contacts[1].point.x).toBe(-1);
		expect(this.contacts[1].point.y).toBe(-0.9);
		expect(this.contacts[1].point.z).toBe(1);
		expect(Math.round(this.contacts[1].penetration*100)/100).toBe(0.1);
		expect(this.contacts[1].restitution).toBe(0.6);
		expect(this.contacts[1].friction).toBe(0.4);
		expect(this.contacts[1].bodies[0]).toBe(this.b1);
		expect(this.contacts[1].bodies[1]).toBe(null);
		expect(this.contacts[2].normal.x).toBe(0);
		expect(this.contacts[2].normal.y).toBe(1);
		expect(this.contacts[2].normal.z).toBe(0);
		expect(this.contacts[2].point.x).toBe(1);
		expect(this.contacts[2].point.y).toBe(-0.9);
		expect(this.contacts[2].point.z).toBe(-1);
		expect(Math.round(this.contacts[2].penetration*100)/100).toBe(0.1);
		expect(this.contacts[2].restitution).toBe(0.6);
		expect(this.contacts[2].friction).toBe(0.4);
		expect(this.contacts[2].bodies[0]).toBe(this.b1);
		expect(this.contacts[2].bodies[1]).toBe(null);
		expect(this.contacts[3].normal.x).toBe(0);
		expect(this.contacts[3].normal.y).toBe(1);
		expect(this.contacts[3].normal.z).toBe(0);
		expect(this.contacts[3].point.x).toBe(-1);
		expect(this.contacts[3].point.y).toBe(-0.9);
		expect(this.contacts[3].point.z).toBe(-1);
		expect(Math.round(this.contacts[3].penetration*100)/100).toBe(0.1);
		expect(this.contacts[3].restitution).toBe(0.6);
		expect(this.contacts[3].friction).toBe(0.4);
		expect(this.contacts[3].bodies[0]).toBe(this.b1);
		expect(this.contacts[3].bodies[1]).toBe(null);

		this.contacts = [];
		off1.makeTranslation(0.9, 0, 0);
		b1 = new Primitives.Box(this.b1, new Mathematics.Vector3D(1, 1, 1), off1);
		p1 = new Primitives.Plane(null, new Mathematics.Vector3D(1, 0, 0), 0);
		expect(this.cd.boxAndHalfSpace(b1, p1, this.data, this.contacts)).toBeTruthy();
		expect(this.contacts.length).toBe(4);
		expect(this.contacts[0].normal.x).toBe(1);
		expect(this.contacts[0].normal.y).toBe(0);
		expect(this.contacts[0].normal.z).toBe(0);
		expect(this.contacts[0].point.x).toBe(0);
		expect(this.contacts[0].point.y).toBe(1);
		expect(this.contacts[0].point.z).toBe(1);
		expect(Math.round(this.contacts[0].penetration*100)/100).toBe(0.1);
		expect(this.contacts[0].restitution).toBe(0.6);
		expect(this.contacts[0].friction).toBe(0.4);
		expect(this.contacts[0].bodies[0]).toBe(this.b1);
		expect(this.contacts[0].bodies[1]).toBe(null);
		expect(this.contacts[1].normal.x).toBe(1);
		expect(this.contacts[1].normal.y).toBe(0);
		expect(this.contacts[1].normal.z).toBe(0);
		expect(this.contacts[1].point.x).toBe(0);
		expect(this.contacts[1].point.y).toBe(-1);
		expect(this.contacts[1].point.z).toBe(1);
		expect(Math.round(this.contacts[1].penetration*100)/100).toBe(0.1);
		expect(this.contacts[1].restitution).toBe(0.6);
		expect(this.contacts[1].friction).toBe(0.4);
		expect(this.contacts[1].bodies[0]).toBe(this.b1);
		expect(this.contacts[1].bodies[1]).toBe(null);
		expect(this.contacts[2].normal.x).toBe(1);
		expect(this.contacts[2].normal.y).toBe(0);
		expect(this.contacts[2].normal.z).toBe(0);
		expect(this.contacts[2].point.x).toBe(0);
		expect(this.contacts[2].point.y).toBe(1);
		expect(this.contacts[2].point.z).toBe(-1);
		expect(Math.round(this.contacts[2].penetration*100)/100).toBe(0.1);
		expect(this.contacts[2].restitution).toBe(0.6);
		expect(this.contacts[2].friction).toBe(0.4);
		expect(this.contacts[2].bodies[0]).toBe(this.b1);
		expect(this.contacts[2].bodies[1]).toBe(null);
		expect(this.contacts[3].normal.x).toBe(1);
		expect(this.contacts[3].normal.y).toBe(0);
		expect(this.contacts[3].normal.z).toBe(0);
		expect(this.contacts[3].point.x).toBe(0);
		expect(this.contacts[3].point.y).toBe(-1);
		expect(this.contacts[3].point.z).toBe(-1);
		expect(Math.round(this.contacts[3].penetration*100)/100).toBe(0.1);
		expect(this.contacts[3].restitution).toBe(0.6);
		expect(this.contacts[3].friction).toBe(0.4);
		expect(this.contacts[3].bodies[0]).toBe(this.b1);
		expect(this.contacts[3].bodies[1]).toBe(null);
	});

	it("should not detect a collision between a box and a half-space when they are too far appart", function() {
		var off1 = new Mathematics.Matrix4D();
		off1.makeTranslation(0, 0, 0);
		var b1 = new Primitives.Box(this.b1, new Mathematics.Vector3D(1, 1, 1), off1);
		var p1 = new Primitives.Plane(null, new Mathematics.Vector3D(0, 1, 0), -1.1);
		expect(this.cd.boxAndHalfSpace(b1, p1, this.data, this.contacts)).toBeFalsy();
		expect(this.contacts.length).toBe(0);
		off1.makeTranslation(1.1, 0, 0);
		b1 = new Primitives.Box(this.b1, new Mathematics.Vector3D(1, 1, 1), off1);
		p1 = new Primitives.Plane(null, new Mathematics.Vector3D(1, 0, 0));
		expect(this.cd.boxAndHalfSpace(b1, p1, this.data, this.contacts)).toBeFalsy();
		expect(this.contacts.length).toBe(0);
	});

	it("should detect a collision between a box and a sphere", function() {
		var off1 = new Mathematics.Matrix4D();
		off1.translate(0, 1, 0);
		var off2 = new Mathematics.Matrix4D();
		off2.translate(0, -0.9, 0);
		var s1 = new Primitives.Sphere(this.b1, 1, off1);
		var b1 = new Primitives.Box(this.b2, new Mathematics.Vector3D(1, 1, 1), off2);
		expect(this.cd.boxAndSphere(b1, s1, this.data, this.contacts)).toBeTruthy();
		expect(this.contacts.length).toBe(1);
		expect(this.contacts[0].normal.x).toBe(0);
		expect(Math.round(this.contacts[0].normal.y)).toBe(-1);
		expect(this.contacts[0].normal.z).toBe(0);
		expect(Math.round(this.contacts[0].point.x)).toBe(0);
		expect(Math.round(this.contacts[0].point.y*10)/10).toBe(0.1);
		expect(Math.round(this.contacts[0].point.z*10)/10).toBe(0);
		expect(Math.round(this.contacts[0].penetration*10)/10).toBe(0.1);
		expect(this.contacts[0].restitution).toBe(0.6);
		expect(this.contacts[0].friction).toBe(0.4);
		expect(this.contacts[0].bodies[0]).toBe(this.b2);
		expect(this.contacts[0].bodies[1]).toBe(this.b1);


		this.contacts = [];
		off1.makeTranslation(2, 0, 0);
		off2.makeTranslation(0.1, 0, 0);
		s1 = new Primitives.Sphere(this.b1, 1, off1);
		b1 = new Primitives.Box(this.b2, new Mathematics.Vector3D(1, 1, 1), off2);
		expect(this.cd.boxAndSphere(b1, s1, this.data, this.contacts)).toBeTruthy();
		expect(this.contacts.length).toBe(1);
		expect(this.contacts[0].normal.x).toBe(-1);
		expect(this.contacts[0].normal.y).toBe(0);
		expect(this.contacts[0].normal.z).toBe(0);
		expect(Math.round(this.contacts[0].point.x)).toBe(1);
		expect(Math.round(this.contacts[0].point.y*10)/10).toBe(0);
		expect(Math.round(this.contacts[0].point.z*10)/10).toBe(0);
		expect(Math.round(this.contacts[0].penetration*10)/10).toBe(0.1);
		expect(this.contacts[0].restitution).toBe(0.6);
		expect(this.contacts[0].friction).toBe(0.4);
		expect(this.contacts[0].bodies[0]).toBe(this.b2);
		expect(this.contacts[0].bodies[1]).toBe(this.b1);
	});


	it("should not detect a collision between a box and a sphere when they are too far appart", function() {
		var off1 = new Mathematics.Matrix4D();
		off1.makeTranslation(0, 1, 0);
		var off2 = new Mathematics.Matrix4D();
		off2.makeTranslation(0, -1.1, 0);

		var s1 = new Primitives.Sphere(this.b1, 1, off1);
		var b1 = new Primitives.Box(this.b1, new Mathematics.Vector3D(1, 1, 1), off2);
		expect(this.cd.boxAndSphere(b1, s1, this.data, this.contacts)).toBeFalsy();
		expect(this.contacts.length).toBe(0);

		off1.makeTranslation(2.1, 0, 0);
		off2.makeTranslation(0, 0, 0);

		s1 = new Primitives.Sphere(this.b1, 1, off1);
		b1 = new Primitives.Box(this.b1, new Mathematics.Vector3D(1, 1, 1), off2);
		expect(this.cd.boxAndSphere(b1, s1, this.data, this.contacts)).toBeFalsy();
		expect(this.contacts.length).toBe(0);
	});

	it("should detect a collision between two boxes", function() {
		var off1 = new Mathematics.Matrix4D();
		off1.makeTranslation(0, -0.9, 0);
		var rot = new Mathematics.Matrix4D();
		rot.makeRotation(Mathematics.Vector3D.Z_AXIS, Math.PI / 6);
		off1.multiply(rot);
		rot.makeRotation(Mathematics.Vector3D.UP, Math.PI / 6);
		off1.multiply(rot);
		var off2 = new Mathematics.Matrix4D();
		off2.makeTranslation(0, 1, 0);

		var b1 = new Primitives.Box(this.b1, new Mathematics.Vector3D(1, 1, 1), off1);
		var b2 = new Primitives.Box(this.b2, new Mathematics.Vector3D(1, 1, 1), off2);
		expect(this.cd.boxAndBox(b1, b2, this.data, this.contacts)).toBeTruthy();
		expect(this.contacts.length).toBe(1);
		expect(this.contacts[0].normal.x).toBe(0);
		expect(this.contacts[0].normal.y).toBe(-1);
		expect(this.contacts[0].normal.z).toBe(0);
		expect(this.contacts[0].point.x).toBe(1);
		expect(Math.round(this.contacts[0].point.y*10)/10).toBe(0);
		expect(this.contacts[0].point.z).toBe(1);
		expect(Math.round(this.contacts[0].penetration*10)/10).toBe(0.1);
		expect(this.contacts[0].restitution).toBe(0.6);
		expect(this.contacts[0].friction).toBe(0.4);
		expect(this.contacts[0].bodies[0]).toBe(this.b1);
		expect(this.contacts[0].bodies[1]).toBe(this.b2);
		//TODO: The expectations are probably wrong and more cases need to be tested.
	});

	it("should not detect a collision between two boxes when they are too far appart", function() {
		var off1 = new Mathematics.Matrix4D();
		off1.makeTranslation(0, -1, 0);
		var off2 = new Mathematics.Matrix4D();
		off2.makeTranslation(0, 1.1, 0);

		var b1 = new Primitives.Box(this.b1, new Mathematics.Vector3D(1, 1, 1), off1);
		var b2 = new Primitives.Box(this.b2, new Mathematics.Vector3D(1, 1, 1), off2);
		expect(this.cd.boxAndBox(b1, b2, this.data, this.contacts)).toBeFalsy();
		expect(this.contacts.length).toBe(0);
		expect(this.cd.boxAndBox(b2, b1, this.data, this.contacts)).toBeFalsy();
		expect(this.contacts.length).toBe(0);

		off1.makeTranslation(0, 0, 0);
		off2.makeTranslation(2.1, 0, 0);

		b1 = new Primitives.Box(this.b1, new Mathematics.Vector3D(1, 1, 1), off1);
		b2 = new Primitives.Box(this.b2, new Mathematics.Vector3D(1, 1, 1), off2);
		expect(this.cd.boxAndBox(b1, b2, this.data, this.contacts)).toBeFalsy();
		expect(this.contacts.length).toBe(0);
		expect(this.cd.boxAndBox(b2, b1, this.data, this.contacts)).toBeFalsy();
		expect(this.contacts.length).toBe(0);
	});

	it("should be able to detect collision between two given bodies, with their primitives set up", function() {
		this.b1 = new RigidBody(new Mathematics.Vector3D(1, 0, 0));
		this.b1.addPrimitives({type: 'box', halfSize: {x: 1, y: 1, z: 1}});
		this.b2 = new RigidBody(new Mathematics.Vector3D(-1, 0, 0));
		this.b2.addPrimitives({type: 'box', halfSize: {x: 1, y: 1, z: 1}});
		expect(this.cd.checkForContacts(this.b1, this.b2, this.data, this.contacts)).toBeTruthy();
		expect(this.contacts.length).toBe(1);
		expect(this.cd.checkForContacts(this.b2, this.b1, this.data, this.contacts)).toBeTruthy();
		expect(this.contacts.length).toBe(2);
	});

	it("should be able to detect collisions between a body and a plane", function() {
		this.b1 = new RigidBody(new Mathematics.Vector3D(1, 0, 0));
		this.b1.addPrimitives({type: 'box', halfSize: {x: 1, y: 1, z: 1}});
		var plane = new Primitives.Plane(null, new Mathematics.Vector3D(0, 1, 0), -1);
		expect(this.cd.checkForContactsWithPlane(this.b1, plane, this.data, this.contacts)).toBeTruthy();
		expect(this.contacts.length).toBe(4);
	});

	it("should not detect collisions between a body and a plane when they are too far appart", function() {
		this.b1 = new RigidBody(new Mathematics.Vector3D(1, 0, 0));
		this.b1.addPrimitives({type: 'box', halfSize: {x: 1, y: 1, z: 1}});
		var plane = new Primitives.Plane(null, new Mathematics.Vector3D(0, 1, 0), -1.1);
		expect(this.cd.checkForContactsWithPlane(this.b1, plane, this.data, this.contacts)).toBeFalsy();
		expect(this.contacts.length).toBe(0);
	});
});