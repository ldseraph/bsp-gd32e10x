set_project("bsp-gd32e10x")

add_rules("mode.release", "mode.debug")

target("bsp-gd32e10x")
    set_kind("static")
    set_toolchains("@gnu-rm")
    add_files("src/*.c")