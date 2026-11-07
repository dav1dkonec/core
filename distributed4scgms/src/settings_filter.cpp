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

#include "settings_filter.h"

#include "deployment_settings.h"
#include "descriptor.h"
#include "logger_init.h"

#include <scgms/iface/DeviceIface.h>
#include <scgms/utils/string_utils.h>

#include <cmath>
#include <format>
#include <iostream>
#include <limits>

namespace scgms_distributed_solver
{
    namespace
    {
        std::string Describe(const deployment_settings::TSettings& settings)
        {
            const auto shown = [](const auto& value)
            {
                return value ? std::format("{}", *value) : std::string{"not set"};
            };

            return std::format(
                "Distributed solver settings read from the configuration: address {}, expected"
                " workers {}, cache dir {}, connect timeout {}. They are used only by the"
                " distributed solver of this library, which the console selects with --solver_id,"
                " not through the configuration.",
                shown(settings.controller_address), shown(settings.expected_workers),
                shown(settings.cache_dir),
                settings.worker_connect_timeout_s
                    ? std::format("{} s", *settings.worker_connect_timeout_s)
                    : std::string{"not set"});
        }
    }

    CSettings_Filter::CSettings_Filter(scgms::IFilter* output) : CBase_Filter(output)
    {
    }

    HRESULT CSettings_Filter::Do_Execute(scgms::UDevice_Event event)
    {
        return mOutput.Send(event);
    }

    HRESULT CSettings_Filter::Do_Configure(scgms::SFilter_Configuration configuration,
                                           refcnt::Swstr_list& error_description)
    {
        deployment_settings::TSettings settings{};

        // None of these can mean anything when left empty, so an empty value stands for "not set"
        // and the solver falls back to its environment variable and then to its default.
        const std::wstring address = configuration.Read_String(rsController_Address);
        if (!address.empty())
        {
            if (address.find(L"://") == std::wstring::npos)
            {
                error_description.push(std::wstring{L"Controller address lacks a transport prefix: "} + address);
                return E_INVALIDARG;
            }

            settings.controller_address = Narrow_WString(address);
        }

        const int64_t workers = configuration.Read_Int(rsExpected_Workers);
        if (workers != std::numeric_limits<int64_t>::max())
        {
            if (workers < 1)
            {
                error_description.push(std::wstring{L"Expected workers must be at least one"});
                return E_INVALIDARG;
            }

            settings.expected_workers = static_cast<size_t>(workers);
        }

        const auto cacheDir = configuration.Read_File_Path(rsCache_Dir);
        if (!cacheDir.empty())
        {
            settings.cache_dir = cacheDir.string();
        }

        // The parameter is a rat time, i.e. a fraction of a day; the solver waits in seconds.
        const double timeout = configuration.Read_Double(rsWorker_Connect_Timeout);
        if (!std::isnan(timeout))
        {
            if (timeout <= 0.0)
            {
                error_description.push(std::wstring{L"Worker connect timeout must be positive"});
                return E_INVALIDARG;
            }

            // Rounded to whole milliseconds; the rat time conversion is not exact and a connect
            // timeout has no use for the residue.
            settings.worker_connect_timeout_s = std::round(timeout / scgms::One_Second * 1000.0) / 1000.0;
        }

        deployment_settings::Set(settings);

        // The library logger has no sinks until the solver installs them, so stderr is what the
        // user actually sees when no distributed solver runs.
        const std::string summary = Describe(settings);
        dist_logger()->info("{}", summary);
        std::cerr << summary << std::endl;

        return S_OK;
    }
}
