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
// Sensor Factory
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

#include "sensor.hpp"

class SensorFactory {
   public:
    using SensorCreator = std::function<std::unique_ptr<Sensor>()>;

    static SensorFactory& getInstance();

    void registerSensor(const std::string& type, SensorCreator creator);

    std::unique_ptr<Sensor> createSensor(const std::string& type);

   private:
    std::unordered_map<std::string, SensorCreator> creators;

    // Private constructor for singleton
    SensorFactory() = default;
    SensorFactory(const SensorFactory&) = delete;
    SensorFactory& operator=(const SensorFactory&) = delete;
};
