#include "malicious-node.h"
#include "ns3/log.h"
#include "ns3/attribute.h"
#include "ns3/boolean.h"



namespace ns3 {

NS_LOG_COMPONENT_DEFINE("MaliciousNode");
// Ensure the type is registered
NS_OBJECT_ENSURE_REGISTERED(MaliciousNode);

TypeId MaliciousNode::GetTypeId(void) {
    static TypeId tid = TypeId("ns3::MaliciousNode")
        .SetParent<Node>()
        .SetGroupName("Network")

        .AddConstructor<MaliciousNode>()
        .AddAttribute("IsMalicious",
                      "Indicates whether the node is malicious.",
                      BooleanValue(false),
                      MakeBooleanAccessor(&MaliciousNode::SetIsMalicious,
&MaliciousNode::IsMalicious),
MakeBooleanChecker());
return tid;
}

MaliciousNode::MaliciousNode() : Node(), m_isMalicious(false) {
NS_LOG_FUNCTION(this);
}

MaliciousNode::~MaliciousNode() {
NS_LOG_FUNCTION(this);
// Cleanup code here if needed
}

void MaliciousNode::SetIsMalicious(bool isMalicious) {
NS_LOG_FUNCTION(this << isMalicious);
m_isMalicious = isMalicious;
}
bool MaliciousNode::IsMalicious() const {
NS_LOG_FUNCTION(this);
return m_isMalicious;
}

} // namespace ns3