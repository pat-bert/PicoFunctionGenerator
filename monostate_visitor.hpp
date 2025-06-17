#ifndef MONOSTATE_VISITOR_HPP_INCLUDED
#define MONOSTATE_VISITOR_HPP_INCLUDED

#include <variant>

struct MonostateVisitor
{
    void operator()(std::monostate) const
    {
        // No operation for std::monostate, which represents an empty variant
    }
};

#endif