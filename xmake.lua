set_project("bsp-gd32e10x")

target("bsp-gd32e10x")
    set_kind("static")
    add_files("src/*.c")