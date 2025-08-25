#pragma once

#include <cstring>
#include <iostream>
#include <source_location>

#define DUMP_TO(out, ...) dump::details::dump_variables(out, #__VA_ARGS__, __VA_ARGS__) << "\n" //

#define DUMP(...) DUMP_TO(std::cout, __VA_ARGS__)

// based on https://stackoverflow.com/a/32230306
namespace dump::details
{
    inline auto &dump_source_location(std::ostream &out, const std::source_location location = std::source_location::current())
    {
        return out << location.file_name() << ':' << location.line();
    }

    auto &dump_variables(std::ostream &out, const char *labels, auto &&first, auto &&...rest)
    {
        if constexpr (sizeof...(rest) == 0)
        {
            return out << labels << ": " << std::forward<decltype(first)>(first);
        }
        else
        {
            const auto end = strchr(labels, ',');
            return dump_variables(
                out.write(labels, end - labels) << ": " << std::forward<decltype(first)>(first) << ", ",
                end + 1,
                std::forward<decltype(rest)>(rest)...);
        }
    }
}
