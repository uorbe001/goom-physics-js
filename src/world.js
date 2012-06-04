var BodyForceRegistry = require("./body_force_registry"), CollisionDetector = require("./collision_detector"), RigidBody = require("./rigid_body"),
	ContactResolver = require("./contact_resolver"), BoundingVolumeHierarchyNode = require("./bounding_volume_hierarchy_node"),
	BoundingSphere = require("./bounding_sphere"), Primitives = require("./primitives"), Mathematics = require("goom-math");
/**
	Creates a World for the physics simulation.
	@class This class holds the different rigid bodies and the force generator/rigid body
	registry. Will also be used to run the physics simulations.
	@exports World as Physics.World
	@property {Array} rigidBodies The array with the different rigid bodies in the world.
	@property {Array} planes The array with the different planes in the world.
	@property {BodyForceRegistry} registry The registry holding all the rigid body and force
	generator pairs.
	@property {Physics.CollisionDetector} collisionDetector This is used to detect collisions between different primitives.
	@property {Physics.ContactResolver} contactResolver The contact resolver is used for contact resolution.
	@property {Physics.BoundingVolumeHierarchyNode} boundingVolumeHierarchy This will be the root node of the BVH, used for broad-phase collision detection.
*/
function World() {
	this.rigidBodies = [];
	this.planes = [];
	this.registry = new BodyForceRegistry();
	this.collisionDetector = new CollisionDetector();
	this.contactResolver = new ContactResolver();
	this.boundingVolumeHierarchy = null;
	this.__potentialContacts = [];
	this.__contacts = [];
}

/**
	Processes all the physics in the world, updating the state of the different objects and calculating internal data before.
	@param {Number} duration Duration of the applied force in seconds.
*/
World.prototype.update = function(duration) {
	//Update the forces generated by force generators
	this.registry.updateForces(duration);

	//Integrate the bodies
	for (var i = 0, len = this.rigidBodies.length; i < len; i++) {
		body = this.rigidBodies[i];
		body.integrate(duration);
	}

	//Look up the bvh for potential contacts (broad-phase detection)
	this.boundingVolumeHierarchy.potentialContacts(this.__potentialContacts);
	//TODO: Contact restitution and friction data CANNOT be hardcoded. This is just a temporary thing.
	data = {restitution: 0, friction: 0};

	var potential_contact;
	//Check each potential contact (narrow-phase detection)
	for (i = 0, len = this.__potentialContacts.length; i < len; i++) {
		potential_contact = this.__potentialContacts[i];
		this.collisionDetector.checkForContacts(potential_contact[0], potential_contact[1], data, this.__contacts);
	}

	var plane, body, j, len2;
	//Check against the world boundaries
	for (i = 0, len = this.planes.length; i < len; i++) {
		plane = this.planes[i];
		for (j = 0, len2 = this.rigidBodies.length; j < len2; j++) {
			body = this.rigidBodies[j];
			this.collisionDetector.checkForContactsWithPlane(body, plane, data, this.__contacts);
		}
	}

	//Contact resolution.
	this.contactResolver.resolve(this.__contacts, duration);
	
	//Clean up
	this.collisionDetector.clearCache();
	this.__contacts.length = 0;
	this.__potentialContacts.length = 0;
};

/**
	Adds a body to the list of rigid bodies in the world and if the second parameter is given, the list of primitives for that body is added.
	@param {JSON} data A description of the body.
*/
World.prototype.addBody = function(data) {
	var body = new RigidBody();
	//Set data
	if (data.id) body.id = data.id;
	if (data.static) body.isStatic = true;
	if (data.position) body.position.set(data.position.x, data.position.y, data.position.z);
	if (data.orientation) body.orientation.set(data.orientation.r, data.orientation.i, data.orientation.j, data.orientation.k);
	if (data.weight) body.setMass(data.weight);
	if (data.inerial_tensor) body.setInertiaTensorCoefficients(data.inerial_tensor[0], data.inerial_tensor[1], data.inerial_tensor[2]);
	//Add the body to the rigid body list.
	this.rigidBodies.push(body);

	//This part only applies to bodies with primitives.
	if (!data.primitives) return body;

	body.addPrimitives(data.primitives);
	var bounding_sphere = new BoundingSphere(body.position, 0);
	bounding_sphere.fitPrimitives(data.primitives);

	//Create a BVH node and add a bounding sphere with the body to it
	if (this.boundingVolumeHierarchy !== null && this.boundingVolumeHierarchy !== undefined)
		this.boundingVolumeHierarchy.insert(body, bounding_sphere);
	else
		this.boundingVolumeHierarchy = new BoundingVolumeHierarchyNode(null, body, bounding_sphere);

	return body;
};


/**
	Returns the body with the given id.
	@param {String} id The id of the body to look for.
	@returns The body with the given id, null if not found.
*/
World.prototype.findBody = function(id) {
	for (var i = this.rigidBodies.length - 1; i >= 0; i--) {
		if (this.rigidBodies[i].id === id) return this.rigidBodies[i];
	}

	return null;
};

/**
	Adds a half plane to the physics world, all rigid bodies will be checked against it per frame.
	@param {Array} data An array or a single element describing the plane.
*/
World.prototype.addPlanes = function(data) {
	if (!(data instanceof Array)) {
		this.planes.push(new Primitives.Plane(this, new Mathematics.Vector3D(data.normal.x, data.normal.y, data.normal.z), data.offset !== null && data.offset !== undefined ? data.offset : null));
		return;
	}

	var one_plane_data;
	//We've got an array
	for (var i = 0, len = data.length; i < len; i++) {
		one_plane_data = data[i];
		this.planes.push(new Primitives.Plane(this, new Mathematics.Vector3D(one_plane_data.normal.x, one_plane_data.normal.y, one_plane_data.normal.z), one_plane_data.offset !== null && one_plane_data.offset !== undefined ? one_plane_data.offset : null));
	}
};

/**
	Registers a force generator to affect a body.
	@param {Physics.RigidBody} body The rigid body to be affected by the given force generator.
	@param {Physics.ForceGenerator} force_generator The force generator affecting the given body.
*/
World.prototype.registerBodyAffectedByForceGenerator = function(body, force_generator) {
	this.registry.add(body, force_generator);
};
/**
	Removes a force generator and affected body pair from the registry.
	@param {Physics.RigidBody} body The rigid body affected by the given force generator.
	@param {Physics.ForceGenerator} force_generator The force generator affecting the given body.
*/
World.prototype.unregisterBodyAffectedByForceGenerator = function(body, force_generator) {
	this.registry.remove(body, force_generator);
};

module.exports = World;