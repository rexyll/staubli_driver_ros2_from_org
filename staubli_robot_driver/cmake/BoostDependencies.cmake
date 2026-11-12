# Ensure the imported Boost::system target exists before ament loads the
# generated exported targets for staubli_robot_driver.
# ament_export_dependencies(Boost) invokes find_package(Boost) without
# forwarding the required "system" component, which does not guarantee that
# Boost::system is defined for downstream consumers.
find_package(Boost REQUIRED COMPONENTS system)
