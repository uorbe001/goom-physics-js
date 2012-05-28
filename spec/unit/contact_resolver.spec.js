var Mathematics = require("goom-math"), Contact = require("../../src/contact"), RigidBody = require("../../src/rigid_body"), ContactResolver = require("../../src/contact_resolver");

describe("Physics.ContactResolver", function() {
	beforeEach(function() {
		this.cr = new ContactResolver();
		this.b1 = new RigidBody();
	});

	it("should resolve contacts(?)", function() {
		var contacts;
		contacts = [];
		contacts.push(new Contact(new Mathematics.Vector3D(1.5, 1.5, 1), new Mathematics.Vector3D(0, -1, 0), 0.5));
		contacts[0].setContactData(this.b1, null, 0.6, 0.4);
		this.cr.resolve(contacts, 0.1);
	});
});