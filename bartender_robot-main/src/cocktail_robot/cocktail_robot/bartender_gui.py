import tkinter as tk
import os
from PIL import Image, ImageTk, ImageEnhance

MAX_AMOUNT = 70
INGREDIENTS = ["Tequila", "Blue juice", "Red juice", "Lime", "Cherry"]


def make_recipe_details(recipes_dict):
    details = {
        "표지": {
            "name": "",
            "desc": "",
            "quote": "",
            "use_ice": False,
            "use_aging": False,
            "use_shaker": False,
        },
        "Margarita": {
            "name": "마가리타",
            "desc": "데킬라 베이스의 상쾌한 칵테일.",
            "quote": '"신선하고 상큼하게~"',
            "use_ice": True,
            "use_aging": False,
            "use_shaker": True,
        },
        "China Red": {
            "name": "차이나 레드",
            "desc": "레드 주스를 활용한 붉은색 칵테일.",
            "quote": '"강렬한 첫인상!"',
            "use_ice": True,
            "use_aging": True,
            "use_shaker": False,
        }
    }
    for recipe in recipes_dict:
        if recipe not in details:
            details[recipe] = {
                "name": recipe,
                "desc": f"{recipe}의 설명이 없습니다.",
                "quote": "",
                "use_ice": False,
                "use_aging": False,
                "use_shaker": False,
            }
    return details

class BartenderGUI(tk.Tk):
    def __init__(self, recipe_names, recipes_dict, robot_action_callback=None):
        super().__init__()
        self.title("Cyberpunk Bartender Dashboard")
        self.geometry("970x540")
        self.configure(bg="#181929")
        self.BASE_DIR = os.path.dirname(os.path.abspath(__file__))

        self.logo_img_orig = None
        self.logo_img = None

        self.recipe_names = recipe_names
        self.recipes_dict = recipes_dict
        self.current_page = 0

        self.recipe_details = make_recipe_details(recipes_dict)
        self.robot_action_callback = robot_action_callback

        self.bar_canvases = []
        self.bar_ids = []

        self.build_ui()
        self.update_screen()
        self.bind("<Configure>", self.on_resize)

    def set_status_msg(self, msg):
        self.run_status_label['text'] = msg

    def build_ui(self):
        self.left = tk.Frame(self, width=410, height=400, bg="#222", bd=0)
        self.cocktail_img_label = tk.Label(self.left)
        self.left.pack(side=tk.LEFT, fill=tk.Y, padx=18, pady=28)
        self.cocktail_img_label.place_forget()
        self.left.pack_propagate(0)

        

        # 로고라벨: self.left 기준에 붙임 (카드 위 아님)
        self.logo_label = tk.Label(self.left, bg="#222")
        self.logo_label.place_forget()

        # 카드 프레임(표지 아닐 때만 노출)
        self.card = tk.Frame(self.left, bg="#111", bd=4, relief="ridge")
        self.card.pack(padx=12, pady=5, fill=tk.BOTH, expand=True)

        self.title_label = tk.Label(self.card, text="", fg="#fff", bg="#111", font=("Consolas", 16, "bold"))
        self.title_label.pack(anchor="w", padx=12, pady=(12,2))
        self.desc_label = tk.Label(self.card, text="", fg="#ccc", bg="#111", font=("Consolas", 12), justify="left", wraplength=370)
        self.desc_label.pack(anchor="w", padx=12, pady=2)
        self.quote_label = tk.Label(self.card, text="", fg="#8cf", bg="#111", font=("Consolas", 11, "italic"))
        self.quote_label.pack(anchor="w", padx=18, pady=(8, 12))

        self.nav = tk.Frame(self.left, bg="#111")
        self.nav.pack(side=tk.BOTTOM, fill=tk.X)

        self.nav.columnconfigure(0, weight=1)
        self.nav.columnconfigure(1, weight=2)
        self.nav.columnconfigure(2, weight=1)

        btn_prev = tk.Button(self.nav, text="◀", width=4, command=self.prev_page)
        btn_next = tk.Button(self.nav, text="▶", width=4, command=self.next_page)
        self.page_label = tk.Label(self.nav, text="1/{}".format(len(self.recipe_names)),
                                bg="#111", fg="#fff", font=("Consolas", 11))

        btn_prev.grid(row=0, column=0, sticky="e", padx=(20, 0), pady=4)
        self.page_label.grid(row=0, column=1, sticky="nsew")
        btn_next.grid(row=0, column=2, sticky="w", padx=(0, 20), pady=4)

        right = tk.Frame(self, width=500, height=480, bg="#19153b")
        right.pack(side=tk.RIGHT, fill=tk.BOTH, padx=8, pady=28)
        right.pack_propagate(0)

        self.slot_frame = tk.Frame(right, bg="#19153b")
        self.slot_frame.pack(pady=(10,0))
        self.bar_canvases = []
        self.bar_ids = []
        for ing in INGREDIENTS:
            f = tk.Frame(self.slot_frame, bg="#19153b")
            f.pack(anchor="w")
            tk.Label(f, text=ing, fg="#ffd700", bg="#19153b", width=12, anchor="w").pack(side=tk.LEFT)
            bar = tk.Canvas(f, width=56, height=20, bg="#1c1c35", highlightthickness=0)
            bar.pack(side=tk.LEFT, padx=2)
            fill_id = bar.create_rectangle(2, 2, 2, 18, fill="#f22", outline="")
            bar.create_rectangle(2, 2, 54, 18, fill="", outline="#888")
            self.bar_canvases.append(bar)
            self.bar_ids.append(fill_id)

        self.run_status_label = tk.Label(
            right,
            text="",
            fg="#fff",
            bg="#19153b",
            font=("Consolas", 14),
            anchor="center",
            pady=8)
        self.run_status_label.pack(fill=tk.X, padx=12, pady=(12,4))

        state_frame = tk.Frame(right, bg="#19153b")
        state_frame.pack(pady=(20,0))

        self._ice_img = Image.open(os.path.join(self.BASE_DIR, "../image/icesprite_0.png")).resize((32,32))
        self._aging_img = Image.open(os.path.join(self.BASE_DIR, "../image/agesprite_0.png")).resize((60,60))
        self._shaker_img = Image.open(os.path.join(self.BASE_DIR, "../image/shaker_spr_0.png")).resize((60,90))
        self.ice_label = tk.Label(state_frame, bg="#19153b")
        self.ice_label.pack(side=tk.LEFT, padx=8)
        self.aging_label = tk.Label(state_frame, bg="#19153b")
        self.aging_label.pack(side=tk.LEFT, padx=8)
        self.shaker_label = tk.Label(state_frame, bg="#19153b")
        self.shaker_label.pack(side=tk.LEFT, padx=8)

        # btns = tk.Frame(right, bg="#19153b")
        # btns.pack(pady=8)
        # tk.Button(btns, text="중지", width=11, bg="#222", fg="#fff", command=self.reset_page).pack(side=tk.LEFT, padx=18)
        # tk.Button(btns, text="섞기", width=11, bg="#e45", fg="#fff", command=self.mix_cocktail).pack(side=tk.LEFT, padx=18)

        btns = tk.Frame(right, bg="#19153b")
        btns.pack(pady=8)
        tk.Button(btns, text="섞기",width=25, height=1, bg="#e45", fg="#fff", command=self.mix_cocktail).pack(padx=18)

        bottom = tk.Frame(self, bg="#101020", height=40)
        bottom.pack(fill=tk.X, side=tk.BOTTOM)
        self.msg_label = tk.Label(bottom, text="", fg="#fff", bg="#101020", font=("Consolas", 11))
        self.msg_label.pack(anchor="w", padx=80)

    def show_cocktail_image(self, page_key):
        COCKTAIL_IMAGES = {
            "표지": "../image/flaming_moai_spr_0.png",
            "Margarita": "../image/cobalt_velvet_spr_0.png",
            "China Red": "../image/bloom_light_spr_0.png"
        }
        filename = COCKTAIL_IMAGES.get(page_key)
        if not filename:
            self.cocktail_img_label.place_forget()
            return

        # 이미지 경로 생성
        img_path = os.path.join(self.BASE_DIR, filename)
        if not os.path.exists(img_path):
            self.cocktail_img_label.place_forget()
            return
        if not os.path.exists(img_path):
            print("NOT FOUND:", img_path)
            self.cocktail_img_label.place_forget()
            return

        max_w, max_h = 120, 120
        orig = Image.open(img_path)
        img = orig.copy()
        img.thumbnail((max_w, max_h), Image.LANCZOS)
        self._cocktail_img = ImageTk.PhotoImage(img)
        self.cocktail_img_label.config(image=self._cocktail_img)
        self.cocktail_img_label.place(relx=1.0, rely=0.0, anchor="ne", x=-0, y=-0)
        self.cocktail_img_label.lift()

    def update_screen(self):
        total_pages = len(self.recipe_names)
        self.page_label['text'] = "{}/{}".format(self.current_page + 1, total_pages)
        recipe_name = self.recipe_names[self.current_page]
        info = self.recipe_details.get(recipe_name, self.recipe_details["표지"])
        self.title_label['text'] = info.get("name", "")
        self.desc_label['text'] = info.get("desc", "")
        self.quote_label['text'] = info.get("quote", "")
        self.set_state_icons(
            info.get('use_ice', False),
            info.get('use_aging', False),
            info.get('use_shaker', False)
        )
        self.update_slots()
        self.show_cocktail_image(recipe_name)  # ★ 추가
        self.msg_label['text'] = ""

    def update_slots(self):
        if self.current_page == 0:
            # 표지: 카드 숨기고 로고 표시
            self.card.pack_forget()
            self.show_logo()
        else:
            self.logo_label.place_forget()
            self.card.pack(padx=12, pady=5, fill=tk.BOTH, expand=True)
            # 이하 기존 내용
            self.title_label.pack(anchor="w", padx=12, pady=(12,2))
            self.desc_label.pack(anchor="w", padx=12, pady=2)
            self.quote_label.pack(anchor="w", padx=18, pady=(8,12))

        ingredient_usage = self.get_ingredient_usage()
        for i, ing in enumerate(INGREDIENTS):
            amount = ingredient_usage.get(ing, 0)
            fill_len = int(52 * min(amount, MAX_AMOUNT) / MAX_AMOUNT) if amount > 0 else 0
            self.bar_canvases[i].coords(self.bar_ids[i], 2, 2, 2 + fill_len, 18)

    def show_logo(self):
        # 패널 크기에 맞게 리사이즈 (비율 유지)
        logo_path = os.path.join(self.BASE_DIR, "../image/logo.png")
        try:
            orig = Image.open(logo_path)
        except Exception as e:
            print(f"Logo not found: {logo_path}")
            self.logo_label.config(image="")
            return

        frame_width = self.left.winfo_width() or 410
        frame_height = self.left.winfo_height() or 400
        max_w = int(frame_width * 0.9)
        max_h = int(frame_height * 0.9)
        if max_w < 1: max_w = 180
        if max_h < 1: max_h = 180

        logo = orig.copy()
        logo.thumbnail((max_w, max_h), Image.LANCZOS)
        self.logo_img = ImageTk.PhotoImage(logo)
        self.logo_label.config(image=self.logo_img)
        self.logo_label.place(relx=0.5, rely=0.5, anchor="center")

    def on_resize(self, event):
        if self.current_page == 0:
            self.show_logo()

    def get_ingredient_usage(self):
        ingredient_name_map = {
            "tequila": "Tequila",
            "blue_juice": "Blue juice",
            "red_juice": "Red juice",
            "lime": "Lime",
            "cherry": "Cherry",
        }
        if self.current_page == 0:
            return {ing: 0 for ing in INGREDIENTS}
        recipe_name = self.recipe_names[self.current_page]
        actions = self.recipes_dict.get(recipe_name, [])
        usage = {ing: 0 for ing in INGREDIENTS}
        for action in actions:
            if action.__class__.__name__ == "PourAction":
                key = getattr(action, "ingredient", None)
                amt = getattr(action, "amount", 0)
                if key in ingredient_name_map and key not in ["lime", "cherry"]:
                    gui_name = ingredient_name_map[key]
                    if amt is not None:
                        usage[gui_name] += amt
        for action in actions:
            if action.__class__.__name__ == "GarnishAction":
                key = getattr(action, "topping", None)
                if key in ["lime", "cherry"]:
                    gui_name = ingredient_name_map[key]
                    usage[gui_name] = MAX_AMOUNT
            elif hasattr(action, "topping"):
                key = getattr(action, "topping", None)
                if key in ["lime", "cherry"]:
                    gui_name = ingredient_name_map[key]
                    usage[gui_name] = MAX_AMOUNT
        return usage


    def set_state_icons(self, ice, aging, shaker):
        def enhance_img(img, active):
            if active:
                return ImageTk.PhotoImage(img)
            else:
                enhancer = ImageEnhance.Brightness(img)
                return ImageTk.PhotoImage(enhancer.enhance(0.25))
        ice_img = enhance_img(self._ice_img, ice)
        aging_img = enhance_img(self._aging_img, aging)
        shaker_img = enhance_img(self._shaker_img, shaker)
        self.ice_label.configure(image=ice_img)
        self.ice_label.image = ice_img
        self.aging_label.configure(image=aging_img)
        self.aging_label.image = aging_img
        self.shaker_label.configure(image=shaker_img)
        self.shaker_label.image = shaker_img

    def prev_page(self):
        if self.current_page > 0:
            self.current_page -= 1
            self.update_screen()

    def next_page(self):
        if self.current_page < len(self.recipe_names) - 1:
            self.current_page += 1
            self.update_screen()

    def reset_page(self):
        if self.robot_action_callback:
            self.robot_action_callback("STOP")
        self.update_screen()
        self.msg_label['text'] = "진행 중단. 표지로 돌아갑니다."

    def mix_cocktail(self):
        if self.current_page > 0 and self.robot_action_callback:
            recipe_name = self.recipe_names[self.current_page]
            self.robot_action_callback(recipe_name)
            self.msg_label['text'] = f"로봇: [{recipe_name}] 제조 요청 전송됨!"
        else:
            self.msg_label['text'] = "레시피를 선택하세요!"
