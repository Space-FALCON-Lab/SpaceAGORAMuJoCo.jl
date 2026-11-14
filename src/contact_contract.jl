# Loaded only when core provides the experimental contact contract. Its version
# is checked before loading any native binding or defining any adapter method.
const CL = SpaceAGORA.ContactLoadServices
_contact_contract_check() = CL.require_version(v"0.1.0")
_contact_contract_check()
