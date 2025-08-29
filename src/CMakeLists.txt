set(EGECONTROLSRC MdlDrawTriangle.cc)

add_library(src STATIC ${EGECONTROLSRC})
install(TARGETS src)