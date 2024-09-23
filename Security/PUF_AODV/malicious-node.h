#ifndef MALICIOUS_NODE_H
#define MALICIOUS_NODE_H

#include "ns3/node.h"
#include "ns3/log.h"
#include "ns3/attribute.h"
#include "ns3/boolean.h"

namespace ns3 {

class MaliciousNode : public Node {
public:
    MaliciousNode();
    virtual ~MaliciousNode();  // Declare a virtual destructor

    void SetIsMalicious(bool isMalicious);
    bool IsMalicious() const;

    static TypeId GetTypeId(void);

private:
    bool m_isMalicious;
};

} // namespace ns3

#endif // MALICIOUS_NODE_H

