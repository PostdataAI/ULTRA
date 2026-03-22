#pragma once

#include <iostream>
#include <vector>
#include <string>

#include "../../Geometry/Point.h"
#include "../../../Helpers/IO/Serialization.h"

namespace GTFS {

class Stop {

public:
    Stop(const std::string& stopId = "", const std::string& name = "", const Geometry::Point& coordinates = Geometry::Point(), const std::string& parentStation = "") :
        stopId(stopId),
        name(name),
        coordinates(coordinates),
        parentStation(parentStation) {
    }
    Stop(IO::Deserialization& deserialize) {
        this->deserialize(deserialize);
    }

    inline bool validate() noexcept {
        if (name.empty()) name = "NOT_NAMED";
        return !stopId.empty();
    }

    friend std::ostream& operator<<(std::ostream& out, const Stop& s) {
        return out << "Stop{" << s.stopId << ", " << s.name  << ", " << s.coordinates << "}";
    }

    inline void serialize(IO::Serialization& serialize) const noexcept {
        serialize(stopId, name, coordinates, parentStation);
    }

    inline void deserialize(IO::Deserialization& deserialize) noexcept {
        deserialize(stopId, name, coordinates, parentStation);
    }

public:
    std::string stopId{""};
    std::string name{""};
    Geometry::Point coordinates{};
    std::string parentStation{""};

};

}
