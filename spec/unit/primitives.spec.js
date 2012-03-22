var requirejs = require("requirejs");
requirejs.config({nodeRequire: require});

requirejs(["../../src/primitives", "../../src/rigid_body", "goom-math"], function(Primitives, RigidBody, Mathematics) {
	describe('Physics.Primitives.Primitive', function() {
		it("should hold a body and its offset", function() {
			var body = new RigidBody();
			var offs = new Mathematics.Matrix4D();
			var primitive = new Primitives.Primitive(body, offs);
			expect(primitive.offset.data.length).toBe(16);
			expect(primitive.offset).not.toBe(offs);

			for (i = 0; i <= 15; i++) {
				expect(primitive.offset.data[i]).toBe(offs.data[i]);
			}

			expect(primitive.body).toBe(body);
		});

		it("should construct a primitive with a body and a default offset", function() {
			var body = new RigidBody();
			var primitive = new Primitives.Primitive(body);
			expect(primitive.offset.data.length).toBe(16);
			var idn = new Mathematics.Matrix4D();
			for (i = 0; i <= 15; i++) {
				expect(primitive.offset.data[i]).toBe(idn.data[i]);
			}

			expect(primitive.body).toBe(body);
		});
	});

	describe('Physics.Primitives.Sphere', function() {
		it("should hold a body and its offset", function() {
			var body = new RigidBody();
			var offs = new Mathematics.Matrix4D();
			var primitive = new Primitives.Sphere(body, 1, offs);
			
			expect(primitive.offset.data.length).toBe(16);
			expect(primitive.offset).not.toBe(offs);
			
			for (i = 0; i <= 15; i++) {
				expect(primitive.offset.data[i]).toBe(offs.data[i]);
			}

			expect(primitive.body).toBe(body);
			expect(primitive.radious).toBe(1);
		});
	
		it("should construct a primitive with a body and a default offset", function() {
			var body = new RigidBody();
			var primitive = new Primitives.Sphere(body, 2);
			expect(primitive.offset.data.length).toBe(16);
			var idn = new Mathematics.Matrix4D();
			for (i = 0; i <= 15; i++) {
				expect(primitive.offset.data[i]).toBe(idn.data[i]);
			}
			expect(primitive.body).toBe(body);
			expect(primitive.radious).toBe(2);
		});
	});

	describe('Physics.Primitives.Plane', function() {
		it("should construct a plane with a body, a normal and its offset", function() {
			var body = new RigidBody();
			var offs = 2;
			var normal = new Mathematics.Vector3D(0, 1, 0);
			var primitive = new Primitives.Plane(body, normal, offs);
			expect(primitive.offset).toBe(2);
			expect(primitive.body).toBe(body);
			expect(primitive.normal.x).toBe(0);
			expect(primitive.normal.y).toBe(1);
			expect(primitive.normal.z).toBe(0);
		});

		it("should construct a plane with a body and a default offset", function() {
			var normal = new Mathematics.Vector3D(0, 1, 0);
			var primitive = new Primitives.Plane(null, normal);
			expect(primitive.offset).toBe(0);
			expect(primitive.body).toBe(null);
			expect(primitive.normal.x).toBe(0);
			expect(primitive.normal.y).toBe(1);
			expect(primitive.normal.z).toBe(0);
		});
	});

	describe('Physics.Primitives.Box', function() {
		it("should construct a box with a body, a half-size vector and its offset", function() {
			var body = new RigidBody();
			var offs = new Mathematics.Matrix4D();
			var half_size = new Mathematics.Vector3D(1, 1, 1);
			var primitive = new Primitives.Box(body, half_size, offs);
			expect(primitive.offset).not.toBe(offs);
			
			for (i = 0; i <= 15; i++) {
				expect(primitive.offset.data[i]).toBe(offs.data[i]);
			}

			expect(primitive.body).toBe(body);
			expect(primitive.halfSize).not.toBe(half_size);
			expect(primitive.halfSize.x).toBe(1);
			expect(primitive.halfSize.y).toBe(1);
			expect(primitive.halfSize.z).toBe(1);
		});

		it("should construct a box with a default offset", function() {
			var body = new RigidBody();
			var offs = new Mathematics.Matrix4D();
			var half_size = new Mathematics.Vector3D(1, 1, 1);
			var primitive = new Primitives.Box(body, half_size);

			for (i = 0; i <= 15; i++) {
				expect(primitive.offset.data[i]).toBe(offs.data[i]);
			}

			expect(primitive.body).toBe(body);
			expect(primitive.halfSize).not.toBe(half_size);
			expect(primitive.halfSize.x).toBe(1);
			expect(primitive.halfSize.y).toBe(1);
			expect(primitive.halfSize.z).toBe(1);
		});
	});
});