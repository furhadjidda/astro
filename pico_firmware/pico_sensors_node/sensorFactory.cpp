// Sensor Factory
/*
 *   This file is part of astro.
 *
 *   astro is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   astro is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License v3.0
 *   along with astro.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "sensorFactory.hpp"

#include "sensor.hpp"

SensorFactory& SensorFactory::getInstance() {
    static SensorFactory instance;
    return instance;
}

void SensorFactory::registerSensor(const std::string& type, SensorCreator creator) {
    creators[type] = std::move(creator);
}

std::unique_ptr<Sensor> SensorFactory::createSensor(const std::string& type) {
    if (creators.find(type) != creators.end()) {
        return creators[type]();
    }
    return nullptr;
}