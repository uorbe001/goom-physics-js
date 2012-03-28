if (typeof define !== 'function') {
	var define = require('amdefine')(module);
}

define(["./body_force_registry", "./bounding_sphere", "./bounding_volume_hierarchy_node", "./collision_detector", "./contact",
		"./force_generator", "./gravity", "./intersection_tests", "./primitives", "./rigid_body"],
		function(BodyForceRegistry, BoundingSphere, BoundingVolumeHierarchyNode, CollisionDetector, Contact,
			ForceGenerator, Gravity, IntersectionTests, Primitives, RigidBody) {
	Physics = {};
	Physics.BodyForceRegistry = BodyForceRegistry;
	Physics.BoundingSphere = BoundingSphere;
	Physics.BoundingVolumeHierarchyNode = BoundingVolumeHierarchyNode;
	Physics.CollisionDetector = CollisionDetector;
	Physics.Contact = Contact;
	Physics.ForceGenerator = ForceGenerator;
	Physics.Gravity = Gravity;
	Physics.IntersectionTests = IntersectionTests;
	Physics.Primitives = Primitives;
	Physics.RigidBody = RigidBody;
	return Physics;
});