var BodyForceRegistry = require("../../src/body_force_registry"), CollisionDetector = require("../../src/collision_detector"),
	ContactResolver = require("../../src/contact_resolver"), BoundingVolumeHierarchyNode = require("../../src/bounding_volume_hierarchy_node"),
	BoundingSphere = require("../../src/bounding_sphere"), Primitives = require("../../src/primitives"), Mathematics = require("goom-math"),
	RigidBody = require("../../src/rigid_body"), World = require("../../src/world"), Gravity = require("../../src/gravity");

describe("Physics.World", function() {
	beforeEach(function() {
		this.world = new World();
	});
	
	it("should call integrate and update the forces for all the objects in the world, along with preparing the bodies before", function() {
		var body, body2, fg;
		body = this.world.addBody({
			"static": false,
			"position": {"x": 0, "y": 0, "z": 0},
			"orientation": {"r": 1, "i": 0, "j": 0, "k": 0},
			"weight": 1,
			"inertial_tensor": [1/12, 1/12, 1/12],
			"primitives": [
				{
					"type": "box",
					"halfSize": {"x": 1, "y": 1, "z": 1}//,
					/*"offset":  [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]*/
				}
		]});

		spyOn(body, 'clear').andCallThrough();
		spyOn(body, 'integrate').andCallThrough();
		body2 = this.world.addBody({
			"static": false,
			"position": {"x": 0, "y": 0, "z": 0},
			"orientation": {"r": 1, "i": 0, "j": 0, "k": 0},
			"weight": 1,
			"inertial_tensor": [1/12, 1/12, 1/12],
			"primitives": [
				{
					"type": "sphere",
					"radious": 2
				}
		]});

		spyOn(body2, 'clear').andCallThrough();
		spyOn(body2, 'integrate').andCallThrough();
		spyOn(this.world.collisionDetector, 'checkForContacts').andCallThrough();
		spyOn(this.world.contactResolver, 'resolve').andCallThrough();
		fg = {};
		fg.updateForce = jasmine.createSpy();
		this.world.registry.add(body, fg);
		this.world.update(1);
		expect(body.integrate).toHaveBeenCalled();
		expect(body2.integrate).toHaveBeenCalled();
		expect(fg.updateForce).toHaveBeenCalledWith(body, 1);
		expect(this.world.collisionDetector.checkForContacts).toHaveBeenCalled();
		expect(this.world.contactResolver.resolve).toHaveBeenCalled();
	});
	
	it("should add a rigid body to the list of rigid bodies and if primitives are described, add primitives too", function() {
		var body, body2, body3, that;
		that = this;
		expect(this.world.boundingVolumeHierarchy !== null).toBeFalsy();
		body = this.world.addBody({
			"static": false,
			"position": {"x": 0, "y": 0, "z": 0},
			"orientation": {"r": 1, "i": 0, "j": 0, "k": 0},
			"weight": 1,
			"inertial_tensor": [1/12, 1/12, 1/12],
			"primitives": [
				{
					"type": "box",
					"halfSize": {"x": 1, "y": 1, "z": 1}//,
					/*"offset":  [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]*/
				}
		]});
		expect(this.world.rigidBodies.length).toBe(1);
		body2 = this.world.addBody({
			"static": false,
			"position": {"x": 0, "y": 0, "z": 0},
			"orientation": {"r": 1, "i": 0, "j": 0, "k": 0},
			"weight": 1,
			"inertial_tensor": [1/12, 1/12, 1/12],
			"primitives": [
				{
					"type": "sphere",
					"radious": 2,
					"offset":  [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
				}
		]});

		expect(this.world.rigidBodies.length).toBe(2);
		expect(body2.primitives.length).toBe(1);
		expect(body2.primitives[0] instanceof Primitives.Sphere).toBeTruthy();
		expect(body2.primitives[0].body).toBe(body2);
		expect(this.world.boundingVolumeHierarchy !== null).toBeTruthy();

		body3 = this.world.addBody({
			"static": false,
			"position": {"x": 0, "y": 0, "z": 0},
			"orientation": {"r": 1, "i": 0, "j": 0, "k": 0},
			"weight": 1,
			"inertial_tensor": [1/12, 1/12, 1/12],
			"primitives": [
				{
					"type": "box",
					"halfSize": {"x": 1, "y": 1, "z": 1},
					"offset":  [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 2, 3, 1]
				}
		]});

		expect(this.world.rigidBodies.length).toBe(3);
		expect(body3.primitives.length).toBe(1);
		expect(body3.primitives[0] instanceof Primitives.Box).toBeTruthy();
		expect(body3.primitives[0].body).toBe(body3);
		expect(this.world.boundingVolumeHierarchy.children[0] !== null).toBeTruthy();
		expect(this.world.boundingVolumeHierarchy.children[1] !== null).toBeTruthy();
	});

	it("should add a body and force to the registry and be able to remove it later", function() {
		var body, fg;
		body = new RigidBody();
		fg = new Gravity(new Mathematics.Vector3D(0, -10, 0));
		this.world.registerBodyAffectedByForceGenerator(body, fg);
		expect(this.world.registry.registrations.length).toBe(1);
		expect(this.world.registry.registrations[0].body).toBe(body);
		expect(this.world.registry.registrations[0].forceGenerator).toBe(fg);
		this.world.unregisterBodyAffectedByForceGenerator(body, fg);
		expect(this.world.registry.registrations.length).toBe(0);
	});

	it("should add planes to the planes in the world", function() {
		this.world.addPlanes({normal: {x: 0,	y: 1,z: 0},	offset: 10});
		expect(this.world.planes.length).toBe(1);
		expect(this.world.planes[0] instanceof Primitives.Plane).toBeTruthy();
		expect(this.world.planes[0].offset).toBe(10);
		expect(this.world.planes[0].normal.x).toBe(0);
		expect(this.world.planes[0].normal.y).toBe(1);
		expect(this.world.planes[0].normal.z).toBe(0);
		this.world.addPlanes([{normal: {x: 1, y: 0,z: 0}, offset: 50 }, { normal: {x: 0, y: 0, z: 1}, offset: 10 }]);
		expect(this.world.planes.length).toBe(3);
		expect(this.world.planes[1] instanceof Primitives.Plane).toBeTruthy();
		expect(this.world.planes[2] instanceof Primitives.Plane).toBeTruthy();
		expect(this.world.planes[1].offset).toBe(50);
		expect(this.world.planes[1].normal.x).toBe(1);
		expect(this.world.planes[1].normal.y).toBe(0);
		expect(this.world.planes[1].normal.z).toBe(0);
		expect(this.world.planes[2].offset).toBe(10);
		expect(this.world.planes[2].normal.x).toBe(0);
		expect(this.world.planes[2].normal.y).toBe(0);
		expect(this.world.planes[2].normal.z).toBe(1);
	});
});