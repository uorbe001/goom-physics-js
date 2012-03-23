var requirejs = require("requirejs");
requirejs.config({nodeRequire: require});

requirejs(["goom-math", "../../src/bounding_volume_hierarchy_node", "../../src/bounding_sphere", "../../src/rigid_body"], function(Mathematics, BoundingVolumeHierarchyNode, BoundingSphere, RigidBody) {
	describe("Physics.BoundingVolumeHierarchyNode", function() {
		beforeEach(function() {
			this.node = new BoundingVolumeHierarchyNode();
		});

		it("should tell whether a node is a leaf node or not", function() {
			expect(this.node.isLeaf()).toBeFalsy();
			this.node.body = new RigidBody();
			expect(this.node.isLeaf()).toBeTruthy();
		});

		it("should tell whether a node is a root node or not", function() {
			expect(this.node.isRoot()).toBeTruthy();
			var n = new BoundingVolumeHierarchyNode(this.node);
			expect(n.isRoot()).toBeFalsy();
			expect(this.node.isRoot()).toBeTruthy();
		});

		it("should check whether the node overlaps another node or not", function() {
			var volume = new BoundingSphere();
			spyOn(volume, "overlaps").andCallThrough();
			var node2 = new BoundingVolumeHierarchyNode(null, null, volume);
			this.node.volume = volume;
			expect(this.node.overlaps(node2)).toBeTruthy();
			expect(volume.overlaps).toHaveBeenCalledWith(volume);
		});

		it("should back out of getting contacts when the node is a leaf node", function() {
			var contacts = [];
			this.node.body = new RigidBody();
			var contact_count = this.node.potentialContacts(contacts, 1);
			expect(contact_count).toBe(0);
		});

		it("should back out of getting contacts when the limit is 0", function() {
			var contacts = [];
			var contact_count = this.node.potentialContacts(contacts, 0);
			expect(contact_count).toBe(0);
		});

		it("should return no contacts when children are not overlapping", function() {
			var contacts = [];
			var node2 = new BoundingVolumeHierarchyNode();
			//Lets fake it.
			this.node.overlaps = function() {
				return false;
			};
			spyOn(this.node, 'overlaps').andCallThrough();
			var contact_count = this.node.potentialContactsWith(node2, contacts, 1);
			expect(this.node.overlaps).toHaveBeenCalledWith(node2);
			expect(contact_count).toBe(0);
		});

		it("should return an array with the contacts when two children are leaf and are overlapping", function() {
			var contacts = [];
			var node2 = new BoundingVolumeHierarchyNode();
			//Lets fake it.
			this.node.overlaps = function() {
				return true;
			};
			spyOn(this.node, 'overlaps').andCallThrough();
			node2.body = new RigidBody();
			this.node.body = new RigidBody();
			var contact_count = this.node.potentialContactsWith(node2, contacts, 10);
			expect(this.node.overlaps).toHaveBeenCalledWith(node2);
			expect(contact_count).toBe(1);
			expect(contacts[0].length).toBe(2);
			expect(contacts[0][0]).toBe(this.node.body);
			expect(contacts[0][1]).toBe(node2.body);
		});

		it("should recurse into the the local node when the other one is a leaf node", function() {
			var contacts = [];
			var node2 = new BoundingVolumeHierarchyNode(null, new RigidBody());
			//Lets fake it.
			this.node.overlaps = function() {
				return true;
			};
			this.node.children[0] = new BoundingVolumeHierarchyNode(null, new RigidBody());
			//Lets fake it.
			this.node.children[0].overlaps = function() {
				return true;
			};

			this.node.children[1] = new BoundingVolumeHierarchyNode(null, new RigidBody());
			//Lets fake it.
			this.node.children[1].overlaps = function() {
				return true;
			};

			spyOn(this.node.children[1], 'potentialContactsWith').andCallThrough();
			spyOn(this.node.children[0], 'potentialContactsWith').andCallThrough();
			var contact_count = this.node.potentialContactsWith(node2, contacts, 10);
			expect(this.node.children[0].potentialContactsWith).toHaveBeenCalled();
			expect(this.node.children[1].potentialContactsWith).toHaveBeenCalled();
			expect(contact_count).toBe(2);
			expect(contacts[0].length).toBe(2);
			expect(contacts[1].length).toBe(2);
			expect(contacts[0][0]).toBe(this.node.children[0].body);
			expect(contacts[0][1]).toBe(node2.body);
			expect(contacts[1][0]).toBe(this.node.children[1].body);
			expect(contacts[1][1]).toBe(node2.body);
		});

		it("should recurse into the other node when this is a leaf node ", function() {
			var contacts = [];
			var node2 = new BoundingVolumeHierarchyNode();
			//Lets fake it.
			this.node.overlaps = function() {
				return true;
			};

			this.node.body = new RigidBody();
			spyOn(this.node, 'potentialContactsWith').andCallThrough();
			node2.children[0] = new BoundingVolumeHierarchyNode(null, new RigidBody());
			node2.children[1] = new BoundingVolumeHierarchyNode(null, new RigidBody());
			var contact_count = this.node.potentialContactsWith(node2, contacts, 10);

			expect(this.node.potentialContactsWith).toHaveBeenCalled();
			expect(contacts.length).toBe(2);
			expect(contact_count).toBe(2);
			expect(contacts[0].length).toBe(2);
			expect(contacts[1].length).toBe(2);
			expect(contacts[0][0]).toBe(this.node.body);
			expect(contacts[0][1]).toBe(node2.children[0].body);
			expect(contacts[1][0]).toBe(this.node.body);
			expect(contacts[1][1]).toBe(node2.children[1].body);
		});

		it("should insert a new node as a children and its body inside another children when the node is a leaf node", function() {
			var b = new RigidBody();
			var b2 = new RigidBody();
			var bounding_sphere = new BoundingSphere();
			var bounding_sphere2 = new BoundingSphere();
			this.node.body = b;
			this.node.volume = bounding_sphere;
			expect(this.node.isLeaf()).toBeTruthy();
			this.node.insert(b2, bounding_sphere2);
			expect(this.node.children.length).toBe(2);
			expect(this.node.isLeaf()).toBeFalsy();
			expect(this.node.children[0].body).toBe(b);
			expect(this.node.children[0].volume.position.x).toBe(0);
			expect(this.node.children[0].volume.position.y).toBe(0);
			expect(this.node.children[0].volume.position.z).toBe(0);
			expect(this.node.children[0].volume.radious).toBe(1);
			expect(this.node.children[1].body).toBe(b2);
			expect(this.node.children[1].volume).toBe(bounding_sphere2);
			expect(this.node.volume.growth(this.node.children[0].volume) <= 0).toBeTruthy();
			expect(this.node.volume.growth(this.node.children[1].volume) <= 0).toBeTruthy();
		});

		it("should grow the child node with the least growth and insert the new body inside it when the node isn't a leaf node", function() {
			var b = new RigidBody();
			var b2 = new RigidBody();
			var b3 = new RigidBody();
			var bounding_sphere = new BoundingSphere();
			var bounding_sphere2 = new BoundingSphere(new Mathematics.Vector3D(1, 1, 1), 2);
			var bounding_sphere3 = new BoundingSphere(new Mathematics.Vector3D(0.5, 0.5, 0.5), 1);
			this.node.children[0] = new BoundingVolumeHierarchyNode(this.node, b, bounding_sphere);
			this.node.children[1] = new BoundingVolumeHierarchyNode(this.node, b2, bounding_sphere2);

			expect(this.node.isLeaf()).toBeFalsy();
			this.node.insert(b3, bounding_sphere3);
			expect(this.node.children[1].children[0].parent).toBe(this.node.children[1]);
			expect(this.node.children[1].children[0].body).toBe(b2);
			expect(this.node.children[1].children[0].volume.position.x).toBe(1);
			expect(this.node.children[1].children[0].volume.position.y).toBe(1);
			expect(this.node.children[1].children[0].volume.position.z).toBe(1);
			expect(this.node.children[1].children[0].volume.radious).toBe(2);
			expect(this.node.children[1].children[1].body).toBe(b3);
			expect(this.node.children[1].children[1].volume).toBe(bounding_sphere3);
			expect(this.node.children[0].body).toBe(b);
			expect(this.node.children[0].volume).toBe(bounding_sphere);
			expect(this.node.children[1].volume.growth(this.node.children[1].children[0].volume) <= 0).toBeTruthy();
			expect(this.node.children[1].volume.growth(this.node.children[1].children[1].volume) <= 0).toBeTruthy();
		});

		it("should free the node's children when a node is freed", function() {
			var b = new RigidBody();
			var b2 = new RigidBody();
			var bounding_sphere = new BoundingSphere();
			var bounding_sphere2 = new BoundingSphere(new Mathematics.Vector3D(1, 1, 1), 2);
			this.node.children[0] = new BoundingVolumeHierarchyNode(this.node, b, bounding_sphere);
			this.node.children[1] = new BoundingVolumeHierarchyNode(this.node, b2, bounding_sphere2);
			this.node.free();
			expect(this.node.children[0] === null).toBeTruthy();
			expect(this.node.children[1] === null).toBeTruthy();
		});

		it("the tree should be reestructured when a non-root node is freed", function() {
			var b = new RigidBody();
			var b2 = new RigidBody();
			var bounding_sphere = new BoundingSphere();
			var bounding_sphere2 = new BoundingSphere(new Mathematics.Vector3D(1, 1, 1), 2);
			this.node.children[0] = new BoundingVolumeHierarchyNode(this.node, b, bounding_sphere);
			this.node.children[1] = new BoundingVolumeHierarchyNode(this.node, b2, bounding_sphere2);
			this.node.children[0].free();
			expect(this.node.children[0] === null).toBeTruthy();
			expect(this.node.children[1] === null).toBeTruthy();
			expect(this.node.body).toBe(b2);
			expect(this.node.volume.size()).toBe(bounding_sphere2.size());
			expect(this.node.volume.position).toBe(bounding_sphere2.position);
		});

		it("should move the nodes to the new positions when a bounding volume's position changes", function() {
			var b = new RigidBody();
			var b2 = new RigidBody();
			var b3 = new RigidBody();
			var v0 = new BoundingSphere();
			this.node = new BoundingVolumeHierarchyNode(null, b3, v0);
			var bounding_sphere = new BoundingSphere(new Mathematics.Vector3D(0, -2, 0), 1);
			var bounding_sphere2 = new BoundingSphere(new Mathematics.Vector3D(0, 1, 0), 1);
			this.node.insert(b, bounding_sphere);
			expect(this.node.children[0].volume.position.x).toBe(0);
			expect(this.node.children[0].volume.position.y).toBe(0);
			expect(this.node.children[0].volume.position.z).toBe(0);
			expect(this.node.children[0].volume.radious).toBe(1);
			expect(this.node.children[0].body).toBe(b3);
			expect(this.node.children[1].volume).toBe(bounding_sphere);
			expect(this.node.children[1].body).toBe(b);

			this.node.insert(b2, bounding_sphere2);
			expect(this.node.children[0].children[1].volume).toBe(bounding_sphere2);
			expect(this.node.children[0].children[1].body).toBe(b2);
			expect(this.node.children[0].children[0].volume.position.x).toBe(0);
			expect(this.node.children[0].children[0].volume.position.y).toBe(0);
			expect(this.node.children[0].children[0].volume.position.z).toBe(0);
			expect(this.node.children[0].children[0].volume.radious).toBe(1);
			expect(this.node.children[0].children[0].body).toBe(b3);
			expect(this.node.children[1].volume).toBe(bounding_sphere);
			expect(this.node.children[1].body).toBe(b);

			b2.position.y = 0;
			b2.boundingVolumeHierarchyNode.updateHierarchy();
			expect(this.node.children[0].children[1].volume).toBe(bounding_sphere2);
			expect(this.node.children[0].children[1].body).toBe(b2);
			
			b2.position.y = -3;
			b2.boundingVolumeHierarchyNode.updateHierarchy();
			expect(this.node.children[1].children[1].volume).toBe(bounding_sphere2);
			expect(this.node.children[1].children[1].body).toBe(b2);
		});

		it("should insert nodes coming from the lower ends of the tree backwards", function() {
			var b = new RigidBody();
			var b2 = new RigidBody();
			var b3 = new RigidBody();
			var v = new BoundingSphere(new Mathematics.Vector3D(1, -3, 0), 1);
			this.node = new BoundingVolumeHierarchyNode(null, new RigidBody(), new BoundingSphere());
			var bounding_sphere = new BoundingSphere(new Mathematics.Vector3D(1, -2, 0), 1);
			var bounding_sphere2 = new BoundingSphere(new Mathematics.Vector3D(1, 1, 0), 1);
			this.node.insert(b, bounding_sphere);
			this.node.insert(b2, bounding_sphere2);
			this.node.children[0].reinsert(b3, v);
			expect(this.node.children[1].children[1].volume).toBe(v);
			expect(this.node.children[1].children[1].body).toBe(b3);
		});
	});
});
