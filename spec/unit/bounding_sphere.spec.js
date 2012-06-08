var Mathematics = require("goom-math-js"), BoundingSphere = require("../../src/bounding_sphere");

describe("Physics.BoundingSphere", function() {
	beforeEach(function() {
		this.s = new BoundingSphere();
	});

	it("should create a bounding sphere containing the two given spheres", function() {
		var s2 = new BoundingSphere(new Mathematics.Vector3D(1, 0, 0), 1);
		var s3 = BoundingSphere.createfromSpheres(this.s, s2);
		expect(s3.radious).toBe(1.5);
		expect(s3.position.x).toBe(0.5);
		expect(s3.position.y).toBe(0);
		expect(s3.position.z).toBe(0);

		s2 = new BoundingSphere(new Mathematics.Vector3D(1, 0, 0), 2);
		s3 = BoundingSphere.createfromSpheres(this.s, s2);
		expect(s3.radious).toBe(2);
		expect(s3.position.x).toBe(1);
		expect(s3.position.y).toBe(0);
		expect(s3.position.z).toBe(0);

		this.s.radious = 2;
		s2 = new BoundingSphere(new Mathematics.Vector3D(1, 0, 0), 1);
		s3 = BoundingSphere.createfromSpheres(this.s, s2);
		expect(s3.radious).toBe(2);
		expect(s3.position.x).toBe(0);
		expect(s3.position.y).toBe(0);
		expect(s3.position.z).toBe(0);
	});

	it("should make the bounding sphere fit the the two given spheres", function() {
		var s2 = new BoundingSphere(new Mathematics.Vector3D(1, 0, 0), 1);
		var s3 = new BoundingSphere(new Mathematics.Vector3D(1, 0, 0), 1);
		s3.fitSpheres(this.s, s2);
		expect(s3.radious).toBe(1.5);
		expect(s3.position.x).toBe(0.5);
		expect(s3.position.y).toBe(0);
		expect(s3.position.z).toBe(0);

		s2 = new BoundingSphere(new Mathematics.Vector3D(1, 0, 0), 2);
		s3.fitSpheres(this.s, s2);
		expect(s3.radious).toBe(2);
		expect(s3.position.x).toBe(1);
		expect(s3.position.y).toBe(0);
		expect(s3.position.z).toBe(0);

		this.s.radious = 2;
		s2 = new BoundingSphere(new Mathematics.Vector3D(1, 0, 0), 1);
		s3.fitSpheres(this.s, s2);
		expect(s3.radious).toBe(2);
		expect(s3.position.x).toBe(0);
		expect(s3.position.y).toBe(0);
		expect(s3.position.z).toBe(0);
	});

	it('should say the spheres are overlapping when they are', function() {
		var s2 = new BoundingSphere(new Mathematics.Vector3D(2, 0, 0), 1.1);
		expect(this.s.overlaps(s2)).toBeTruthy();
		expect(s2.overlaps(this.s)).toBeTruthy();

		s2 = new BoundingSphere(new Mathematics.Vector3D(0, 2, 0), 1.1);
		expect(this.s.overlaps(s2)).toBeTruthy();
		expect(s2.overlaps(this.s)).toBeTruthy();

		s2 = new BoundingSphere(new Mathematics.Vector3D(0, 0, 2), 1.1);
		expect(this.s.overlaps(s2)).toBeTruthy();
		expect(s2.overlaps(this.s)).toBeTruthy();
	});


	it('should say the spheres are not overlapping when they are not', function() {
		var s2 = new BoundingSphere(new Mathematics.Vector3D(2, 0, 0), 1);
		expect(this.s.overlaps(s2)).toBeFalsy();
		expect(s2.overlaps(this.s)).toBeFalsy();

		s2 = new BoundingSphere(new Mathematics.Vector3D(0, 2, 0), 1);
		expect(this.s.overlaps(s2)).toBeFalsy();
		expect(s2.overlaps(this.s)).toBeFalsy();

		s2 = new BoundingSphere(new Mathematics.Vector3D(0, 0, 2), 1);
		expect(this.s.overlaps(s2)).toBeFalsy();
		expect(s2.overlaps(this.s)).toBeFalsy();
	});

	it("should tell the size the sphere needs to grow to hold the given sphere", function() {
		var s2 = new BoundingSphere(new Mathematics.Vector3D(1, 0, 0), 1);
		expect(this.s.growth(s2)).toBe(1.25);
	});
});