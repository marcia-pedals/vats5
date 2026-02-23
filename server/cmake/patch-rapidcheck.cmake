# Patch rapidcheck for C++26 compatibility.
# Gen.hpp uses std::current_exception() without including <exception>.
file(READ "include/rapidcheck/Gen.hpp" content)
string(FIND "${content}" "#include <exception>" found)
if(found EQUAL -1)
  string(REPLACE "#include <cassert>" "#include <cassert>\n#include <exception>" content "${content}")
  file(WRITE "include/rapidcheck/Gen.hpp" "${content}")
endif()
