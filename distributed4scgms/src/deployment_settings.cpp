/**
 * SmartCGMS - continuous glucose monitoring and controlling framework
 * https://diabetes.zcu.cz/
 *
 * Copyright (c) since 2018 University of West Bohemia.
 *
 * Contact:
 * diabetes@mail.kiv.zcu.cz
 * Medical Informatics, Department of Computer Science and Engineering
 * Faculty of Applied Sciences, University of West Bohemia
 * Univerzitni 8, 301 00 Pilsen
 * Czech Republic
 * 
 * 
 * Purpose of this software:
 * This software is intended to demonstrate work of the diabetes.zcu.cz research
 * group to other scientists, to complement our published papers. It is strictly
 * prohibited to use this software for diagnosis or treatment of any medical condition,
 * without obtaining all required approvals from respective regulatory bodies.
 *
 * Especially, a diabetic patient is warned that unauthorized use of this software
 * may result into severe injure, including death.
 *
 *
 * Licensing terms:
 * Unless required by applicable law or agreed to in writing, software
 * distributed under these license terms is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *
 * a) Without a specific agreement, you are not authorized to make or keep any copies of this file.
 * b) For any use, especially commercial use, you must contact us and obtain specific terms and conditions 
 *    for the use of the software.
 * c) When publishing any derivative work or results obtained using this software, you agree to cite the following paper:
 *    Tomas Koutny and Martin Ubl, "SmartCGMS as a Testbed for a Blood-Glucose Level Prediction and/or 
 *    Control Challenge with (an FDA-Accepted) Diabetic Patient Simulation", Procedia Computer Science,  
 *    Volume 177, pp. 354-362, 2020
 */

#include "deployment_settings.h"

#include <scgms/utils/string_utils.h>

#include <algorithm>
#include <charconv>
#include <cstdlib>

namespace
{
    bool parse_count(const std::string& text, size_t& out)
    {
        unsigned long long value = 0;
        const auto [end, ec] = std::from_chars(text.data(), text.data() + text.size(), value);
        if (ec != std::errc{} || end != text.data() + text.size())
        {
            return false;
        }

        out = static_cast<size_t>(value);
        return true;
    }

    bool parse_seconds(const std::string& text, double& out)
    {
        char* end = nullptr;
        out = std::strtod(text.c_str(), &end);
        return !text.empty() && end == text.c_str() + text.size() && out > 0;
    }

    std::optional<heterogeneity_policy> parse_policy(const std::string& text)
    {
        if (text == "by_population") return heterogeneity_policy::BY_POPULATION;
        if (text == "by_both") return heterogeneity_policy::BY_BOTH;
        if (text == "by_generations") return heterogeneity_policy::BY_GENERATIONS;
        return std::nullopt;
    }
}

std::optional<deployment_settings::TSettings> deployment_settings::Parse(const wchar_t* options, std::string& error)
{
    TSettings settings{};
    if (!options)
    {
        return settings;
    }

    const std::string text = Narrow_WString(options);
    for (size_t start = 0; start <= text.size();)
    {
        const size_t end = std::min(text.find(';', start), text.size());
        const std::string item = text.substr(start, end - start);
        start = end + 1;

        if (item.empty())
        {
            continue;
        }

        const size_t separator = item.find('=');
        if (separator == std::string::npos || separator == 0 || separator + 1 == item.size())
        {
            error = "'" + item + "' is not key=value";
            return std::nullopt;
        }

        const std::string key = item.substr(0, separator);
        const std::string value = item.substr(separator + 1);

        if (key == "controller_address")
        {
            if (value.find("://") == std::string::npos)
            {
                error = "controller_address lacks a transport prefix: " + value;
                return std::nullopt;
            }
            settings.controller_address = value;
        }
        else if (key == "expected_workers")
        {
            size_t workers = 0;
            if (!parse_count(value, workers) || workers == 0)
            {
                error = "expected_workers must be a whole number of at least one: " + value;
                return std::nullopt;
            }
            settings.expected_workers = workers;
        }
        else if (key == "cache_dir")
        {
            settings.cache_dir = value;
        }
        else if (key == "worker_connect_timeout_s")
        {
            double seconds = 0;
            if (!parse_seconds(value, seconds))
            {
                error = "worker_connect_timeout_s must be a positive number of seconds: " + value;
                return std::nullopt;
            }
            settings.worker_connect_timeout_s = seconds;
        }
        else if (key == "cycle_count")
        {
            size_t cycles = 0;
            if (!parse_count(value, cycles) || cycles == 0)
            {
                error = "cycle_count must be a whole number of at least one: " + value;
                return std::nullopt;
            }
            settings.cycle_count = cycles;
        }
        else if (key == "island_cycle_count")
        {
            size_t cycles = 0;
            if (!parse_count(value, cycles) || cycles == 0)
            {
                error = "island_cycle_count must be a whole number of at least one: " + value;
                return std::nullopt;
            }
            settings.island_cycle_count = cycles;
        }
        else if (key == "heterogeneity_policy")
        {
            settings.policy = parse_policy(value);
            if (!settings.policy)
            {
                error = "heterogeneity_policy is by_population, by_both or by_generations: " + value;
                return std::nullopt;
            }
        }
        else
        {
            error = "unknown solver option '" + key + "'";
            return std::nullopt;
        }
    }

    return settings;
}

const char* deployment_settings::Policy_Name(const heterogeneity_policy policy)
{
    switch (policy)
    {
    case heterogeneity_policy::BY_POPULATION: return "by_population";
    case heterogeneity_policy::BY_BOTH: return "by_both";
    case heterogeneity_policy::BY_GENERATIONS: return "by_generations";
    }

    return "unknown";
}
