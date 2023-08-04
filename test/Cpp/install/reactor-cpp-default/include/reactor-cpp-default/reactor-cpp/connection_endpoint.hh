#ifndef REACTOR_CPP_CONNECTION_ENDPOINT_HH
#define REACTOR_CPP_CONNECTION_ENDPOINT_HH

#include <type_traits>
#include "port.hh"

/*
base classes for endpoints for federated comms
*/

namespace reactor {

/*
Downstream inherits from Action bec it produces event on receiver end
*/
template <class T>
class DownstreamEndpoint : public Action<T> {
    protected:
        std::set<Port<T>*> ports_;

        virtual void schedule_this(/*value here*/) {
            if constexpr (std::is_same<T, void>::value) {
                this->schedule();
            } else {
                //this->schedule(std::move(value here));
            }
        }


    public:
        DownstreamEndpoint(const std::string& name, Reactor* container, bool is_logical, Duration min_delay)
        : Action<T>(name, container, is_logical, min_delay) {}
        DownstreamEndpoint(const std::string& name, Environment* environment, bool is_logical, Duration min_delay)
        : Action<T>(name, environment, is_logical, min_delay) {}
 
        void add_port(Port<T>* port) {
            [[maybe_unused]] bool result = ports_.insert(port).second;
            reactor_assert(result);
        }

        void setup() noexcept override {
            Action<T>::setup();

            if constexpr (std::is_same<T, void>::value) {
                for (auto port : this->ports_) {
                    port->set();
                }
            } else {
                for (auto port : this->ports_) {
                    port->set(std::move(this->get()));
                }
            }

        }
};

template <class T>
class UpstreamEndpoint {
    protected: 
        Port<T>* port_ = nullptr;

        virtual PortCallback set_cb() {
            return [this](const BasePort& port) {
                auto& typed_port = reinterpret_cast<const Port<T>&>(port); 
                if constexpr (std::is_same<T, void>::value) {
                    // send void
                } else {
                    // send std::move(typed_port.get());
                }
            };
        }

    public:
        void set_port(Port<T>* port) {
            reactor_assert(port_ == nullptr);
            port_ = port;
            port_->register_set_callback(set_cb());
        }
};

// reactor ns
}
#endif