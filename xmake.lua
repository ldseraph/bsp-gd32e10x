set_defaultplat("cross")

set_project("bsp-gd32e10x")

set_languages("c99")

add_repositories("ldseraph-repo https://github.com/ldseraph/xmake-repo main")

add_requires("gnu-rm")

add_rules("mode.release", "mode.debug")

target("bsp-gd32e10x")
    set_kind("static")
    set_toolchains("@gnu-rm")
    add_files("src/*.c")