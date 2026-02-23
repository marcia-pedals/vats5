// Dynamic per-checkout favicons — each vats5-{suffix} gets a unique icon.

/** Wrap an emoji character in an SVG <text> element. */
function emojiSvg(emoji: string): string {
  return `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100"><rect width="100" height="100" rx="16" fill="#2d2d2d"/><text y="82" x="10" font-size="75">${emoji}</text></svg>`;
}

/** Suffixes with an exact Unicode emoji match. */
const emojiMap: Record<string, string> = {
  apple: "🍎",
  avocado: "🥑",
  banana: "🍌",
  blueberry: "🫐",
  broccoli: "🥦",
  carrot: "🥕",
  cherry: "🍒",
  coconut: "🥥",
  corn: "🌽",
  cucumber: "🥒",
  eggplant: "🍆",
  garlic: "🧄",
  ginger: "🫚",
  grape: "🍇",
  honeydew: "🍈",
  jalapeno: "🌶️",
  kiwi: "🥝",
  lemon: "🍋",
  lettuce: "🥬",
  mango: "🥭",
  melon: "🍈",
  mushroom: "🍄",
  olive: "🫒",
  onion: "🧅",
  orange: "🍊",
  peach: "🍑",
  pear: "🍐",
  pepper: "🫑",
  pineapple: "🍍",
  potato: "🥔",
  strawberry: "🍓",
  tangerine: "🍊",
  tomato: "🍅",
  watermelon: "🍉",
  yam: "🍠",
};

/**
 * Hand-crafted SVG icons for suffixes without emoji.
 * Each is a minimal SVG designed to be recognizable at 32×32.
 * All use viewBox="0 0 100 100".
 */
const customSvgMap: Record<string, string> = {
  apricot: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
    <rect width="100" height="100" rx="16" fill="#2d2d2d"/>
    <circle cx="50" cy="55" r="38" fill="#F4A233"/>
    <ellipse cx="50" cy="55" rx="38" ry="36" fill="#F0922B"/>
    <path d="M50 17 C50 17 48 8 42 5" stroke="#5B8C2A" stroke-width="3" fill="none" stroke-linecap="round"/>
    <ellipse cx="44" cy="6" rx="8" ry="5" fill="#6DAF3A" transform="rotate(-30 44 6)"/>
    <path d="M50 20 Q50 55 50 90" stroke="#E8832A" stroke-width="1.5" fill="none" opacity="0.4"/>
  </svg>`,

  artichoke: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
    <rect width="100" height="100" rx="16" fill="#2d2d2d"/>
    <ellipse cx="50" cy="58" rx="28" ry="35" fill="#7BA05B"/>
    <path d="M30 65 Q50 30 70 65" fill="#8DB86B"/>
    <path d="M33 55 Q50 25 67 55" fill="#9CC87B"/>
    <path d="M36 45 Q50 20 64 45" fill="#ABDA8B"/>
    <path d="M40 37 Q50 18 60 37" fill="#B8E698"/>
    <path d="M46 22 Q50 10 54 22" fill="#6B9B4B"/>
    <line x1="50" y1="10" x2="50" y2="22" stroke="#5B8B3B" stroke-width="3" stroke-linecap="round"/>
  </svg>`,

  arugula: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
    <rect width="100" height="100" rx="16" fill="#2d2d2d"/>
    <path d="M50 90 L50 40" stroke="#5B8C2A" stroke-width="3" fill="none"/>
    <path d="M50 40 Q30 20 25 25 Q20 30 40 45 Z" fill="#4CAF50"/>
    <path d="M50 40 Q70 20 75 25 Q80 30 60 45 Z" fill="#4CAF50"/>
    <path d="M50 50 Q25 35 22 42 Q19 49 45 55 Z" fill="#66BB6A"/>
    <path d="M50 50 Q75 35 78 42 Q81 49 55 55 Z" fill="#66BB6A"/>
    <path d="M50 60 Q30 50 28 55 Q26 60 48 63 Z" fill="#81C784"/>
  </svg>`,

  asparagus: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
    <rect width="100" height="100" rx="16" fill="#2d2d2d"/>
    <rect x="43" y="20" width="14" height="70" rx="7" fill="#6B9B3A"/>
    <path d="M50 20 Q50 10 50 8" stroke="#5B8B2A" stroke-width="4" stroke-linecap="round"/>
    <path d="M43 30 L38 25" stroke="#7BAB4A" stroke-width="2" stroke-linecap="round"/>
    <path d="M57 35 L62 30" stroke="#7BAB4A" stroke-width="2" stroke-linecap="round"/>
    <path d="M43 45 L38 40" stroke="#7BAB4A" stroke-width="2" stroke-linecap="round"/>
    <path d="M57 50 L62 45" stroke="#7BAB4A" stroke-width="2" stroke-linecap="round"/>
    <ellipse cx="50" cy="12" rx="8" ry="12" fill="#5B8B2A"/>
  </svg>`,

  basil: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
    <rect width="100" height="100" rx="16" fill="#2d2d2d"/>
    <line x1="50" y1="90" x2="50" y2="35" stroke="#3E7D1E" stroke-width="3"/>
    <ellipse cx="40" cy="30" rx="18" ry="12" fill="#4CAF50" transform="rotate(-15 40 30)"/>
    <ellipse cx="60" cy="30" rx="18" ry="12" fill="#4CAF50" transform="rotate(15 60 30)"/>
    <ellipse cx="35" cy="50" rx="16" ry="10" fill="#66BB6A" transform="rotate(-25 35 50)"/>
    <ellipse cx="65" cy="50" rx="16" ry="10" fill="#66BB6A" transform="rotate(25 65 50)"/>
    <ellipse cx="50" cy="18" rx="14" ry="10" fill="#388E3C"/>
  </svg>`,

  beet: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
    <rect width="100" height="100" rx="16" fill="#2d2d2d"/>
    <circle cx="50" cy="60" r="32" fill="#8B1A4A"/>
    <ellipse cx="50" cy="58" rx="32" ry="30" fill="#A0225A"/>
    <path d="M50 28 Q48 15 44 8" stroke="#4CAF50" stroke-width="3" fill="none" stroke-linecap="round"/>
    <path d="M50 28 Q55 12 60 5" stroke="#4CAF50" stroke-width="3" fill="none" stroke-linecap="round"/>
    <path d="M50 28 Q42 10 35 12" stroke="#4CAF50" stroke-width="2.5" fill="none" stroke-linecap="round"/>
    <ellipse cx="50" cy="90" rx="5" ry="6" fill="#7A1040"/>
  </svg>`,

  cabbage: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
    <rect width="100" height="100" rx="16" fill="#2d2d2d"/>
    <circle cx="50" cy="55" r="38" fill="#7CB342"/>
    <circle cx="50" cy="55" r="30" fill="#8BC34A"/>
    <circle cx="50" cy="55" r="22" fill="#9CCC65"/>
    <circle cx="50" cy="55" r="14" fill="#AED581"/>
    <circle cx="50" cy="55" r="7" fill="#C5E1A5"/>
  </svg>`,

  cauliflower: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
    <rect width="100" height="100" rx="16" fill="#2d2d2d"/>
    <ellipse cx="35" cy="72" rx="14" ry="8" fill="#4CAF50" transform="rotate(-20 35 72)"/>
    <ellipse cx="65" cy="72" rx="14" ry="8" fill="#4CAF50" transform="rotate(20 65 72)"/>
    <ellipse cx="25" cy="65" rx="12" ry="7" fill="#66BB6A" transform="rotate(-40 25 65)"/>
    <ellipse cx="75" cy="65" rx="12" ry="7" fill="#66BB6A" transform="rotate(40 75 65)"/>
    <circle cx="50" cy="45" r="18" fill="#F5F5DC"/>
    <circle cx="35" cy="50" r="15" fill="#FFFFF0"/>
    <circle cx="65" cy="50" r="15" fill="#FFFFF0"/>
    <circle cx="42" cy="38" r="13" fill="#FFF8E7"/>
    <circle cx="58" cy="38" r="13" fill="#FFF8E7"/>
    <circle cx="50" cy="32" r="12" fill="#FFFEF5"/>
    <circle cx="30" cy="55" r="10" fill="#F5F0D0"/>
    <circle cx="70" cy="55" r="10" fill="#F5F0D0"/>
  </svg>`,

  celery: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
    <rect width="100" height="100" rx="16" fill="#2d2d2d"/>
    <path d="M40 90 Q38 50 35 20" stroke="#8BC34A" stroke-width="8" fill="none" stroke-linecap="round"/>
    <path d="M50 90 Q50 50 50 20" stroke="#9CCC65" stroke-width="8" fill="none" stroke-linecap="round"/>
    <path d="M60 90 Q62 50 65 20" stroke="#8BC34A" stroke-width="8" fill="none" stroke-linecap="round"/>
    <ellipse cx="35" cy="15" rx="8" ry="10" fill="#66BB6A"/>
    <ellipse cx="50" cy="12" rx="8" ry="10" fill="#66BB6A"/>
    <ellipse cx="65" cy="15" rx="8" ry="10" fill="#66BB6A"/>
  </svg>`,

  cilantro: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
    <rect width="100" height="100" rx="16" fill="#2d2d2d"/>
    <line x1="50" y1="90" x2="50" y2="45" stroke="#3E7D1E" stroke-width="2.5"/>
    <line x1="50" y1="55" x2="30" y2="35" stroke="#3E7D1E" stroke-width="2"/>
    <line x1="50" y1="55" x2="70" y2="35" stroke="#3E7D1E" stroke-width="2"/>
    <circle cx="28" cy="30" r="4" fill="#4CAF50"/><circle cx="22" cy="25" r="4" fill="#4CAF50"/>
    <circle cx="34" cy="25" r="4" fill="#4CAF50"/><circle cx="28" cy="20" r="4" fill="#66BB6A"/>
    <circle cx="72" cy="30" r="4" fill="#4CAF50"/><circle cx="78" cy="25" r="4" fill="#4CAF50"/>
    <circle cx="66" cy="25" r="4" fill="#4CAF50"/><circle cx="72" cy="20" r="4" fill="#66BB6A"/>
    <circle cx="50" cy="40" r="4" fill="#4CAF50"/><circle cx="44" cy="35" r="4" fill="#4CAF50"/>
    <circle cx="56" cy="35" r="4" fill="#4CAF50"/><circle cx="50" cy="30" r="4" fill="#66BB6A"/>
  </svg>`,

  cranberry: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
    <rect width="100" height="100" rx="16" fill="#2d2d2d"/>
    <ellipse cx="38" cy="55" rx="18" ry="15" fill="#C62828"/>
    <ellipse cx="62" cy="55" rx="18" ry="15" fill="#D32F2F"/>
    <ellipse cx="50" cy="45" rx="16" ry="14" fill="#E53935"/>
    <circle cx="38" cy="50" r="2" fill="#EF9A9A" opacity="0.6"/>
    <circle cx="62" cy="50" r="2" fill="#EF9A9A" opacity="0.6"/>
    <circle cx="50" cy="40" r="2" fill="#EF9A9A" opacity="0.6"/>
  </svg>`,

  date: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
    <rect width="100" height="100" rx="16" fill="#2d2d2d"/>
    <ellipse cx="50" cy="55" rx="18" ry="30" fill="#5D4037"/>
    <ellipse cx="50" cy="55" rx="16" ry="28" fill="#6D4C41"/>
    <path d="M50 25 Q48 18 45 15" stroke="#4CAF50" stroke-width="2" fill="none" stroke-linecap="round"/>
    <ellipse cx="43" cy="14" rx="6" ry="4" fill="#66BB6A"/>
    <path d="M42 50 Q50 55 58 50" stroke="#4E342E" stroke-width="1" fill="none" opacity="0.5"/>
  </svg>`,

  dill: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
    <rect width="100" height="100" rx="16" fill="#2d2d2d"/>
    <line x1="50" y1="90" x2="50" y2="25" stroke="#558B2F" stroke-width="2.5"/>
    <g stroke="#7CB342" stroke-width="1.5" fill="none" stroke-linecap="round">
      <path d="M50 30 Q35 25 30 20"/><path d="M50 30 Q65 25 70 20"/>
      <path d="M50 40 Q33 35 27 30"/><path d="M50 40 Q67 35 73 30"/>
      <path d="M50 50 Q35 45 30 40"/><path d="M50 50 Q65 45 70 40"/>
      <path d="M50 60 Q37 55 33 50"/><path d="M50 60 Q63 55 67 50"/>
      <path d="M50 70 Q40 65 37 60"/><path d="M50 70 Q60 65 63 60"/>
    </g>
    <circle cx="35" cy="15" r="3" fill="#CDDC39"/>
    <circle cx="65" cy="15" r="3" fill="#CDDC39"/>
    <circle cx="50" cy="20" r="3" fill="#CDDC39"/>
  </svg>`,

  endive: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
    <rect width="100" height="100" rx="16" fill="#2d2d2d"/>
    <ellipse cx="50" cy="55" rx="30" ry="38" fill="#AED581"/>
    <path d="M50 17 Q45 50 35 85" stroke="#9CCC65" stroke-width="2" fill="none"/>
    <path d="M50 17 Q55 50 65 85" stroke="#9CCC65" stroke-width="2" fill="none"/>
    <path d="M50 17 Q40 40 30 75" stroke="#C5E1A5" stroke-width="1.5" fill="none"/>
    <path d="M50 17 Q60 40 70 75" stroke="#C5E1A5" stroke-width="1.5" fill="none"/>
    <path d="M38 25 Q50 17 62 25" fill="#8BC34A"/>
  </svg>`,

  fennel: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
    <rect width="100" height="100" rx="16" fill="#2d2d2d"/>
    <ellipse cx="50" cy="70" rx="22" ry="25" fill="#F0F4C3"/>
    <ellipse cx="50" cy="68" rx="18" ry="20" fill="#F5F7DC"/>
    <path d="M42 45 Q40 25 35 10" stroke="#8BC34A" stroke-width="3" fill="none" stroke-linecap="round"/>
    <path d="M50 45 Q50 25 50 8" stroke="#7CB342" stroke-width="3" fill="none" stroke-linecap="round"/>
    <path d="M58 45 Q60 25 65 10" stroke="#8BC34A" stroke-width="3" fill="none" stroke-linecap="round"/>
    <g stroke="#AED581" stroke-width="1" fill="none">
      <path d="M35 10 Q25 8 20 12"/><path d="M35 10 Q30 5 25 3"/>
      <path d="M65 10 Q75 8 80 12"/><path d="M65 10 Q70 5 75 3"/>
    </g>
  </svg>`,

  fig: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
    <rect width="100" height="100" rx="16" fill="#2d2d2d"/>
    <path d="M50 20 Q75 20 78 55 Q80 80 50 90 Q20 80 22 55 Q25 20 50 20Z" fill="#6A1B9A"/>
    <path d="M50 20 Q70 22 72 52 Q74 75 50 84 Q26 75 28 52 Q30 22 50 20Z" fill="#7B1FA2"/>
    <path d="M50 20 Q48 12 46 8" stroke="#5B8C2A" stroke-width="3" fill="none" stroke-linecap="round"/>
    <ellipse cx="44" cy="8" rx="7" ry="4" fill="#66BB6A" transform="rotate(-20 44 8)"/>
  </svg>`,

  grapefruit: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
    <rect width="100" height="100" rx="16" fill="#2d2d2d"/>
    <circle cx="50" cy="52" r="38" fill="#FF8A65"/>
    <circle cx="50" cy="52" r="35" fill="#FFAB91"/>
    <circle cx="50" cy="52" r="10" fill="#FFCCBC"/>
    <line x1="50" y1="17" x2="50" y2="87" stroke="#FFCCBC" stroke-width="2" opacity="0.5"/>
    <line x1="15" y1="52" x2="85" y2="52" stroke="#FFCCBC" stroke-width="2" opacity="0.5"/>
    <line x1="25" y1="27" x2="75" y2="77" stroke="#FFCCBC" stroke-width="2" opacity="0.5"/>
    <line x1="75" y1="27" x2="25" y2="77" stroke="#FFCCBC" stroke-width="2" opacity="0.5"/>
    <path d="M48 14 Q46 8 43 5" stroke="#4CAF50" stroke-width="2.5" fill="none" stroke-linecap="round"/>
  </svg>`,

  guava: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
    <rect width="100" height="100" rx="16" fill="#2d2d2d"/>
    <circle cx="50" cy="55" r="35" fill="#8BC34A"/>
    <circle cx="50" cy="55" r="30" fill="#9CCC65"/>
    <circle cx="50" cy="55" r="18" fill="#F48FB1"/>
    <circle cx="45" cy="52" r="2" fill="#FFCDD2"/>
    <circle cx="55" cy="52" r="2" fill="#FFCDD2"/>
    <circle cx="50" cy="58" r="2" fill="#FFCDD2"/>
    <path d="M50 20 Q48 12 45 8" stroke="#558B2F" stroke-width="2.5" fill="none" stroke-linecap="round"/>
  </svg>`,

  jicama: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
    <rect width="100" height="100" rx="16" fill="#2d2d2d"/>
    <path d="M50 15 Q80 20 85 55 Q88 85 50 90 Q12 85 15 55 Q20 20 50 15Z" fill="#D7C4A0"/>
    <path d="M50 18 Q75 22 80 53 Q83 80 50 85 Q17 80 20 53 Q25 22 50 18Z" fill="#E8DCBE"/>
    <path d="M50 15 Q48 8 50 5" stroke="#8D6E63" stroke-width="2.5" fill="none" stroke-linecap="round"/>
  </svg>`,

  kale: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
    <rect width="100" height="100" rx="16" fill="#2d2d2d"/>
    <line x1="50" y1="90" x2="50" y2="40" stroke="#33691E" stroke-width="4"/>
    <path d="M50 40 Q20 30 15 40 Q10 50 25 55 Q15 60 20 70 Q25 80 40 75 Q35 85 50 85 Q65 85 60 75 Q75 80 80 70 Q85 60 75 55 Q90 50 85 40 Q80 30 50 40Z" fill="#2E7D32"/>
    <path d="M50 40 Q30 35 28 42 Q26 50 35 52 Q28 58 32 65 Q36 72 45 68 Q42 78 50 78 Q58 78 55 68 Q64 72 68 65 Q72 58 65 52 Q74 50 72 42 Q70 35 50 40Z" fill="#388E3C"/>
  </svg>`,

  kohlrabi: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
    <rect width="100" height="100" rx="16" fill="#2d2d2d"/>
    <circle cx="50" cy="60" r="30" fill="#C5CAE9"/>
    <circle cx="50" cy="58" r="28" fill="#D1C4E9"/>
    <path d="M42 30 Q38 15 32 8" stroke="#66BB6A" stroke-width="3" fill="none" stroke-linecap="round"/>
    <path d="M50 30 Q50 12 52 5" stroke="#66BB6A" stroke-width="3" fill="none" stroke-linecap="round"/>
    <path d="M58 30 Q62 15 68 8" stroke="#66BB6A" stroke-width="3" fill="none" stroke-linecap="round"/>
    <ellipse cx="30" cy="7" rx="9" ry="5" fill="#81C784" transform="rotate(-15 30 7)"/>
    <ellipse cx="52" cy="4" rx="9" ry="5" fill="#81C784"/>
    <ellipse cx="70" cy="7" rx="9" ry="5" fill="#81C784" transform="rotate(15 70 7)"/>
  </svg>`,

  kumquat: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
    <rect width="100" height="100" rx="16" fill="#2d2d2d"/>
    <ellipse cx="50" cy="55" rx="20" ry="28" fill="#FF9800"/>
    <ellipse cx="50" cy="53" rx="18" ry="26" fill="#FFA726"/>
    <path d="M50 27 Q48 18 45 14" stroke="#4CAF50" stroke-width="2.5" fill="none" stroke-linecap="round"/>
    <ellipse cx="43" cy="13" rx="7" ry="4" fill="#66BB6A" transform="rotate(-20 43 13)"/>
  </svg>`,

  leek: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
    <rect width="100" height="100" rx="16" fill="#2d2d2d"/>
    <rect x="44" y="40" width="12" height="50" rx="6" fill="#F5F5F5"/>
    <rect x="44" y="40" width="12" height="25" rx="3" fill="#E8F5E9"/>
    <path d="M44 40 Q30 20 25 5" stroke="#4CAF50" stroke-width="6" fill="none" stroke-linecap="round"/>
    <path d="M56 40 Q70 20 75 5" stroke="#4CAF50" stroke-width="6" fill="none" stroke-linecap="round"/>
    <path d="M50 40 Q50 15 50 3" stroke="#388E3C" stroke-width="5" fill="none" stroke-linecap="round"/>
  </svg>`,

  lime: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
    <rect width="100" height="100" rx="16" fill="#2d2d2d"/>
    <circle cx="50" cy="52" r="38" fill="#689F38"/>
    <circle cx="50" cy="52" r="35" fill="#7CB342"/>
    <circle cx="50" cy="52" r="8" fill="#C5E1A5"/>
    <line x1="50" y1="17" x2="50" y2="87" stroke="#C5E1A5" stroke-width="1.5" opacity="0.4"/>
    <line x1="15" y1="52" x2="85" y2="52" stroke="#C5E1A5" stroke-width="1.5" opacity="0.4"/>
    <line x1="25" y1="27" x2="75" y2="77" stroke="#C5E1A5" stroke-width="1.5" opacity="0.4"/>
    <line x1="75" y1="27" x2="25" y2="77" stroke="#C5E1A5" stroke-width="1.5" opacity="0.4"/>
    <circle cx="45" cy="14" r="3" fill="#558B2F"/>
  </svg>`,

  lychee: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
    <rect width="100" height="100" rx="16" fill="#2d2d2d"/>
    <circle cx="50" cy="55" r="32" fill="#E91E63"/>
    <circle cx="50" cy="53" r="30" fill="#EC407A"/>
    <g fill="none" stroke="#C2185B" stroke-width="0.8" opacity="0.5">
      <circle cx="38" cy="42" r="6"/><circle cx="55" cy="38" r="6"/><circle cx="65" cy="50" r="6"/>
      <circle cx="60" cy="65" r="6"/><circle cx="42" cy="68" r="6"/><circle cx="35" cy="55" r="6"/>
      <circle cx="50" cy="55" r="6"/>
    </g>
    <path d="M50 23 Q48 15 45 10" stroke="#4CAF50" stroke-width="2.5" fill="none" stroke-linecap="round"/>
    <ellipse cx="43" cy="9" rx="7" ry="4" fill="#66BB6A" transform="rotate(-15 43 9)"/>
  </svg>`,

  mint: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
    <rect width="100" height="100" rx="16" fill="#2d2d2d"/>
    <line x1="50" y1="90" x2="50" y2="30" stroke="#2E7D32" stroke-width="2.5"/>
    <ellipse cx="38" cy="35" rx="15" ry="10" fill="#4CAF50" transform="rotate(-20 38 35)"/>
    <ellipse cx="62" cy="35" rx="15" ry="10" fill="#4CAF50" transform="rotate(20 62 35)"/>
    <ellipse cx="35" cy="55" rx="14" ry="9" fill="#66BB6A" transform="rotate(-25 35 55)"/>
    <ellipse cx="65" cy="55" rx="14" ry="9" fill="#66BB6A" transform="rotate(25 65 55)"/>
    <ellipse cx="38" cy="73" rx="12" ry="8" fill="#81C784" transform="rotate(-20 38 73)"/>
    <ellipse cx="62" cy="73" rx="12" ry="8" fill="#81C784" transform="rotate(20 62 73)"/>
  </svg>`,

  nectarine: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
    <rect width="100" height="100" rx="16" fill="#2d2d2d"/>
    <circle cx="50" cy="55" r="35" fill="#EF6C00"/>
    <circle cx="50" cy="53" r="33" fill="#F57C00"/>
    <path d="M50 20 Q48 12 45 8" stroke="#5B8C2A" stroke-width="2.5" fill="none" stroke-linecap="round"/>
    <ellipse cx="43" cy="7" rx="7" ry="4" fill="#66BB6A" transform="rotate(-20 43 7)"/>
    <path d="M50 22 Q50 55 50 88" stroke="#E65100" stroke-width="1.5" fill="none" opacity="0.3"/>
  </svg>`,

  nutmeg: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
    <rect width="100" height="100" rx="16" fill="#2d2d2d"/>
    <ellipse cx="50" cy="55" rx="22" ry="28" fill="#795548"/>
    <ellipse cx="50" cy="53" rx="20" ry="26" fill="#8D6E63"/>
    <path d="M50 27 Q42 30 38 55 Q36 75 50 80" stroke="#6D4C41" stroke-width="1.5" fill="none" opacity="0.5"/>
    <path d="M50 27 Q58 30 62 55 Q64 75 50 80" stroke="#6D4C41" stroke-width="1.5" fill="none" opacity="0.5"/>
    <path d="M50 27 Q50 55 50 80" stroke="#6D4C41" stroke-width="1" fill="none" opacity="0.4"/>
  </svg>`,

  okra: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
    <rect width="100" height="100" rx="16" fill="#2d2d2d"/>
    <path d="M50 10 Q60 20 58 50 Q56 80 50 95 Q44 80 42 50 Q40 20 50 10Z" fill="#558B2F"/>
    <path d="M50 10 Q56 22 55 50 Q54 78 50 90 Q46 78 45 50 Q44 22 50 10Z" fill="#689F38"/>
    <path d="M50 10 Q48 5 46 3" stroke="#33691E" stroke-width="2.5" fill="none" stroke-linecap="round"/>
    <line x1="50" y1="15" x2="50" y2="85" stroke="#7CB342" stroke-width="1" opacity="0.4"/>
  </svg>`,

  oregano: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
    <rect width="100" height="100" rx="16" fill="#2d2d2d"/>
    <line x1="50" y1="90" x2="50" y2="30" stroke="#33691E" stroke-width="2.5"/>
    <line x1="50" y1="50" x2="30" y2="40" stroke="#33691E" stroke-width="2"/>
    <line x1="50" y1="50" x2="70" y2="40" stroke="#33691E" stroke-width="2"/>
    <circle cx="50" cy="25" r="8" fill="#4CAF50"/>
    <circle cx="28" cy="36" r="7" fill="#66BB6A"/>
    <circle cx="72" cy="36" r="7" fill="#66BB6A"/>
    <line x1="50" y1="65" x2="35" y2="58" stroke="#33691E" stroke-width="1.5"/>
    <line x1="50" y1="65" x2="65" y2="58" stroke="#33691E" stroke-width="1.5"/>
    <circle cx="33" cy="55" r="6" fill="#81C784"/>
    <circle cx="67" cy="55" r="6" fill="#81C784"/>
  </svg>`,

  papaya: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
    <rect width="100" height="100" rx="16" fill="#2d2d2d"/>
    <ellipse cx="50" cy="52" rx="28" ry="38" fill="#FF9800"/>
    <ellipse cx="50" cy="52" rx="25" ry="35" fill="#FFB74D"/>
    <ellipse cx="50" cy="52" rx="12" ry="22" fill="#FF7043"/>
    <circle cx="47" cy="42" r="2.5" fill="#37474F"/>
    <circle cx="53" cy="42" r="2.5" fill="#37474F"/>
    <circle cx="50" cy="50" r="2.5" fill="#37474F"/>
    <circle cx="47" cy="58" r="2.5" fill="#37474F"/>
    <circle cx="53" cy="58" r="2.5" fill="#37474F"/>
    <path d="M50 14 Q48 8 44 5" stroke="#4CAF50" stroke-width="2.5" fill="none" stroke-linecap="round"/>
  </svg>`,

  parsley: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
    <rect width="100" height="100" rx="16" fill="#2d2d2d"/>
    <line x1="50" y1="90" x2="50" y2="45" stroke="#2E7D32" stroke-width="2.5"/>
    <line x1="50" y1="55" x2="30" y2="30" stroke="#2E7D32" stroke-width="2"/>
    <line x1="50" y1="55" x2="70" y2="30" stroke="#2E7D32" stroke-width="2"/>
    <g fill="#4CAF50">
      <circle cx="25" cy="25" r="5"/><circle cx="30" cy="20" r="5"/><circle cx="35" cy="25" r="5"/>
      <circle cx="65" cy="25" r="5"/><circle cx="70" cy="20" r="5"/><circle cx="75" cy="25" r="5"/>
      <circle cx="45" cy="40" r="5"/><circle cx="50" cy="35" r="5"/><circle cx="55" cy="40" r="5"/>
    </g>
  </svg>`,

  parsnip: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
    <rect width="100" height="100" rx="16" fill="#2d2d2d"/>
    <path d="M50 15 Q62 20 60 45 Q58 70 50 95 Q42 70 40 45 Q38 20 50 15Z" fill="#F5DEB3"/>
    <path d="M50 15 Q58 22 57 45 Q55 68 50 90 Q45 68 43 45 Q42 22 50 15Z" fill="#FAEBD7"/>
    <path d="M50 15 Q48 8 45 5" stroke="#4CAF50" stroke-width="2.5" fill="none" stroke-linecap="round"/>
    <ellipse cx="43" cy="5" rx="7" ry="4" fill="#66BB6A" transform="rotate(-15 43 5)"/>
  </svg>`,

  persimmon: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
    <rect width="100" height="100" rx="16" fill="#2d2d2d"/>
    <circle cx="50" cy="55" r="35" fill="#E65100"/>
    <ellipse cx="50" cy="53" rx="35" ry="33" fill="#EF6C00"/>
    <path d="M32 25 Q50 18 68 25" fill="#4CAF50"/>
    <path d="M38 22 Q50 16 62 22" fill="#66BB6A"/>
    <path d="M44 20 Q50 15 56 20" fill="#81C784"/>
    <circle cx="50" cy="17" r="3" fill="#33691E"/>
  </svg>`,

  plum: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
    <rect width="100" height="100" rx="16" fill="#2d2d2d"/>
    <circle cx="50" cy="55" r="34" fill="#6A1B9A"/>
    <circle cx="50" cy="53" r="32" fill="#7B1FA2"/>
    <path d="M50 21 Q48 12 46 8" stroke="#5B8C2A" stroke-width="2.5" fill="none" stroke-linecap="round"/>
    <ellipse cx="44" cy="7" rx="7" ry="4" fill="#66BB6A" transform="rotate(-20 44 7)"/>
    <path d="M50 23 Q50 55 50 87" stroke="#4A148C" stroke-width="1.5" fill="none" opacity="0.3"/>
    <ellipse cx="40" cy="42" rx="6" ry="10" fill="#9C27B0" opacity="0.3" transform="rotate(-15 40 42)"/>
  </svg>`,

  pomegranate: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
    <rect width="100" height="100" rx="16" fill="#2d2d2d"/>
    <circle cx="50" cy="55" r="35" fill="#C62828"/>
    <circle cx="50" cy="53" r="33" fill="#D32F2F"/>
    <polygon points="42,18 50,8 58,18" fill="#E57373"/>
    <polygon points="44,18 50,10 56,18" fill="#EF9A9A"/>
    <circle cx="50" cy="17" r="2" fill="#B71C1C"/>
    <ellipse cx="42" cy="45" rx="4" ry="5" fill="#E53935" opacity="0.4"/>
    <ellipse cx="58" cy="45" rx="4" ry="5" fill="#E53935" opacity="0.4"/>
    <ellipse cx="50" cy="60" rx="4" ry="5" fill="#E53935" opacity="0.4"/>
  </svg>`,

  pumpkin: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
    <rect width="100" height="100" rx="16" fill="#2d2d2d"/>
    <ellipse cx="50" cy="60" rx="38" ry="32" fill="#E65100"/>
    <ellipse cx="35" cy="60" rx="18" ry="30" fill="#EF6C00" opacity="0.7"/>
    <ellipse cx="65" cy="60" rx="18" ry="30" fill="#EF6C00" opacity="0.7"/>
    <ellipse cx="50" cy="60" rx="14" ry="32" fill="#F57C00" opacity="0.5"/>
    <path d="M50 28 Q50 18 50 12" stroke="#4CAF50" stroke-width="4" stroke-linecap="round" fill="none"/>
    <path d="M50 15 Q42 10 38 14" stroke="#4CAF50" stroke-width="2.5" fill="none" stroke-linecap="round"/>
  </svg>`,

  quince: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
    <rect width="100" height="100" rx="16" fill="#2d2d2d"/>
    <circle cx="50" cy="55" r="34" fill="#C8B400"/>
    <ellipse cx="50" cy="57" rx="34" ry="32" fill="#D4C200"/>
    <path d="M50 21 Q48 12 46 8" stroke="#5B8C2A" stroke-width="2.5" fill="none" stroke-linecap="round"/>
    <ellipse cx="44" cy="7" rx="7" ry="4" fill="#66BB6A" transform="rotate(-20 44 7)"/>
    <ellipse cx="50" cy="88" rx="4" ry="3" fill="#B8A600"/>
  </svg>`,

  radish: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
    <rect width="100" height="100" rx="16" fill="#2d2d2d"/>
    <path d="M50 35 Q68 38 65 60 Q62 82 50 92 Q38 82 35 60 Q32 38 50 35Z" fill="#E53935"/>
    <path d="M50 35 Q62 37 60 58 Q58 78 50 86 Q42 78 40 58 Q38 37 50 35Z" fill="#EF5350"/>
    <ellipse cx="50" cy="90" rx="4" ry="6" fill="#FFCCBC"/>
    <path d="M45 35 Q38 18 30 10" stroke="#4CAF50" stroke-width="3" fill="none" stroke-linecap="round"/>
    <path d="M55 35 Q62 18 70 10" stroke="#4CAF50" stroke-width="3" fill="none" stroke-linecap="round"/>
    <ellipse cx="28" cy="9" rx="9" ry="6" fill="#66BB6A" transform="rotate(-15 28 9)"/>
    <ellipse cx="72" cy="9" rx="9" ry="6" fill="#66BB6A" transform="rotate(15 72 9)"/>
  </svg>`,

  raspberry: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
    <rect width="100" height="100" rx="16" fill="#2d2d2d"/>
    <g fill="#C2185B">
      <circle cx="42" cy="40" r="8"/><circle cx="58" cy="40" r="8"/>
      <circle cx="35" cy="52" r="8"/><circle cx="50" cy="50" r="8"/><circle cx="65" cy="52" r="8"/>
      <circle cx="38" cy="64" r="8"/><circle cx="52" cy="65" r="8"/><circle cx="62" cy="63" r="7"/>
      <circle cx="45" cy="75" r="7"/><circle cx="57" cy="74" r="7"/>
    </g>
    <g fill="#E91E63" opacity="0.3">
      <circle cx="42" cy="38" r="3"/><circle cx="58" cy="38" r="3"/>
      <circle cx="50" cy="48" r="3"/><circle cx="38" cy="62" r="3"/>
    </g>
    <path d="M50 32 Q48 18 44 12" stroke="#4CAF50" stroke-width="2.5" fill="none" stroke-linecap="round"/>
    <path d="M50 32 Q55 20 60 14" stroke="#4CAF50" stroke-width="2" fill="none" stroke-linecap="round"/>
  </svg>`,

  rhubarb: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
    <rect width="100" height="100" rx="16" fill="#2d2d2d"/>
    <rect x="30" y="30" width="10" height="60" rx="5" fill="#C62828" transform="rotate(-5 35 60)"/>
    <rect x="45" y="28" width="10" height="62" rx="5" fill="#D32F2F"/>
    <rect x="60" y="30" width="10" height="60" rx="5" fill="#C62828" transform="rotate(5 65 60)"/>
    <ellipse cx="35" cy="25" rx="14" ry="10" fill="#4CAF50" transform="rotate(-10 35 25)"/>
    <ellipse cx="50" cy="22" rx="14" ry="10" fill="#66BB6A"/>
    <ellipse cx="65" cy="25" rx="14" ry="10" fill="#4CAF50" transform="rotate(10 65 25)"/>
  </svg>`,

  rosemary: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
    <rect width="100" height="100" rx="16" fill="#2d2d2d"/>
    <line x1="50" y1="90" x2="50" y2="10" stroke="#5D4037" stroke-width="2.5"/>
    <g fill="#2E7D32">
      <ellipse cx="44" cy="20" rx="8" ry="3" transform="rotate(-30 44 20)"/>
      <ellipse cx="56" cy="25" rx="8" ry="3" transform="rotate(30 56 25)"/>
      <ellipse cx="44" cy="32" rx="8" ry="3" transform="rotate(-30 44 32)"/>
      <ellipse cx="56" cy="37" rx="8" ry="3" transform="rotate(30 56 37)"/>
      <ellipse cx="44" cy="44" rx="8" ry="3" transform="rotate(-30 44 44)"/>
      <ellipse cx="56" cy="49" rx="8" ry="3" transform="rotate(30 56 49)"/>
      <ellipse cx="44" cy="56" rx="8" ry="3" transform="rotate(-30 44 56)"/>
      <ellipse cx="56" cy="61" rx="8" ry="3" transform="rotate(30 56 61)"/>
      <ellipse cx="44" cy="68" rx="8" ry="3" transform="rotate(-30 44 68)"/>
      <ellipse cx="56" cy="73" rx="8" ry="3" transform="rotate(30 56 73)"/>
    </g>
  </svg>`,

  rutabaga: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
    <rect width="100" height="100" rx="16" fill="#2d2d2d"/>
    <circle cx="50" cy="55" r="32" fill="#E8C96A"/>
    <ellipse cx="50" cy="45" rx="32" ry="15" fill="#9C27B0" opacity="0.3"/>
    <ellipse cx="50" cy="88" rx="5" ry="8" fill="#D4B85A"/>
    <path d="M45 23 Q40 10 35 5" stroke="#4CAF50" stroke-width="3" fill="none" stroke-linecap="round"/>
    <path d="M55 23 Q60 10 65 5" stroke="#4CAF50" stroke-width="3" fill="none" stroke-linecap="round"/>
    <ellipse cx="33" cy="5" rx="8" ry="5" fill="#66BB6A" transform="rotate(-10 33 5)"/>
    <ellipse cx="67" cy="5" rx="8" ry="5" fill="#66BB6A" transform="rotate(10 67 5)"/>
  </svg>`,

  sage: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
    <rect width="100" height="100" rx="16" fill="#2d2d2d"/>
    <line x1="50" y1="90" x2="50" y2="30" stroke="#5D4037" stroke-width="2.5"/>
    <ellipse cx="38" cy="30" rx="16" ry="8" fill="#78909C" transform="rotate(-15 38 30)"/>
    <ellipse cx="62" cy="35" rx="16" ry="8" fill="#78909C" transform="rotate(15 62 35)"/>
    <ellipse cx="36" cy="50" rx="15" ry="7" fill="#90A4AE" transform="rotate(-20 36 50)"/>
    <ellipse cx="64" cy="55" rx="15" ry="7" fill="#90A4AE" transform="rotate(20 64 55)"/>
    <ellipse cx="38" cy="70" rx="14" ry="7" fill="#B0BEC5" transform="rotate(-15 38 70)"/>
    <ellipse cx="62" cy="75" rx="14" ry="7" fill="#B0BEC5" transform="rotate(15 62 75)"/>
  </svg>`,

  shallot: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
    <rect width="100" height="100" rx="16" fill="#2d2d2d"/>
    <ellipse cx="50" cy="55" rx="22" ry="32" fill="#8D6E63"/>
    <ellipse cx="50" cy="53" rx="20" ry="30" fill="#A1887F"/>
    <path d="M50 23 Q48 12 50 5" stroke="#8BC34A" stroke-width="3" fill="none" stroke-linecap="round"/>
    <ellipse cx="50" cy="87" rx="4" ry="5" fill="#795548"/>
    <path d="M50 25 Q50 55 50 85" stroke="#6D4C41" stroke-width="1" fill="none" opacity="0.3"/>
  </svg>`,

  spinach: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
    <rect width="100" height="100" rx="16" fill="#2d2d2d"/>
    <ellipse cx="50" cy="40" rx="32" ry="22" fill="#2E7D32"/>
    <ellipse cx="50" cy="40" rx="28" ry="18" fill="#388E3C"/>
    <path d="M50 62 L50 90" stroke="#33691E" stroke-width="3" stroke-linecap="round"/>
    <path d="M50 40 Q35 30 25 35" stroke="#1B5E20" stroke-width="1.5" fill="none"/>
    <path d="M50 40 Q65 30 75 35" stroke="#1B5E20" stroke-width="1.5" fill="none"/>
    <path d="M50 40 Q40 35 30 40" stroke="#1B5E20" stroke-width="1.5" fill="none"/>
    <path d="M50 40 Q60 35 70 40" stroke="#1B5E20" stroke-width="1.5" fill="none"/>
  </svg>`,

  squash: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
    <rect width="100" height="100" rx="16" fill="#2d2d2d"/>
    <ellipse cx="50" cy="55" rx="20" ry="35" fill="#F9A825"/>
    <ellipse cx="50" cy="55" rx="16" ry="33" fill="#FBC02D"/>
    <path d="M50 20 Q48 12 50 8" stroke="#4CAF50" stroke-width="3" fill="none" stroke-linecap="round"/>
    <ellipse cx="50" cy="20" rx="10" ry="5" fill="#F57F17" opacity="0.5"/>
    <line x1="50" y1="22" x2="50" y2="88" stroke="#F9A825" stroke-width="1.5" opacity="0.4"/>
  </svg>`,

  thyme: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
    <rect width="100" height="100" rx="16" fill="#2d2d2d"/>
    <path d="M50 90 Q48 50 45 10" stroke="#5D4037" stroke-width="2"/>
    <g fill="#66BB6A">
      <ellipse cx="42" cy="18" rx="5" ry="3" transform="rotate(-20 42 18)"/>
      <ellipse cx="48" cy="18" rx="5" ry="3" transform="rotate(20 48 18)"/>
      <ellipse cx="41" cy="28" rx="5" ry="3" transform="rotate(-20 41 28)"/>
      <ellipse cx="49" cy="28" rx="5" ry="3" transform="rotate(20 49 28)"/>
      <ellipse cx="42" cy="38" rx="5" ry="3" transform="rotate(-20 42 38)"/>
      <ellipse cx="50" cy="38" rx="5" ry="3" transform="rotate(20 50 38)"/>
      <ellipse cx="43" cy="48" rx="5" ry="3" transform="rotate(-20 43 48)"/>
      <ellipse cx="51" cy="48" rx="5" ry="3" transform="rotate(20 51 48)"/>
      <ellipse cx="44" cy="58" rx="5" ry="3" transform="rotate(-20 44 58)"/>
      <ellipse cx="52" cy="58" rx="5" ry="3" transform="rotate(20 52 58)"/>
    </g>
  </svg>`,

  turnip: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
    <rect width="100" height="100" rx="16" fill="#2d2d2d"/>
    <path d="M50 30 Q75 33 72 58 Q68 85 50 92 Q32 85 28 58 Q25 33 50 30Z" fill="#F5F5F5"/>
    <path d="M50 30 Q70 32 68 42 Q65 52 50 48 Q35 52 32 42 Q30 32 50 30Z" fill="#CE93D8" opacity="0.6"/>
    <ellipse cx="50" cy="90" rx="4" ry="6" fill="#EEEEEE"/>
    <path d="M45 30 Q38 15 30 8" stroke="#4CAF50" stroke-width="3" fill="none" stroke-linecap="round"/>
    <path d="M55 30 Q62 15 70 8" stroke="#4CAF50" stroke-width="3" fill="none" stroke-linecap="round"/>
    <ellipse cx="28" cy="7" rx="9" ry="5" fill="#66BB6A" transform="rotate(-15 28 7)"/>
    <ellipse cx="72" cy="7" rx="9" ry="5" fill="#66BB6A" transform="rotate(15 72 7)"/>
  </svg>`,

  zucchini: `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
    <rect width="100" height="100" rx="16" fill="#2d2d2d"/>
    <ellipse cx="50" cy="50" rx="16" ry="40" fill="#2E7D32" transform="rotate(-15 50 50)"/>
    <ellipse cx="50" cy="50" rx="13" ry="38" fill="#388E3C" transform="rotate(-15 50 50)"/>
    <ellipse cx="39" cy="84" rx="6" ry="4" fill="#FDD835" transform="rotate(-15 39 84)"/>
    <circle cx="59" cy="16" r="4" fill="#33691E"/>
  </svg>`,
};

/**
 * Set the page favicon based on the checkout suffix.
 * Extracts the suffix from __CHECKOUT_NAME__ (e.g. "vats5-cauliflower" → "cauliflower")
 * and applies the matching SVG as a data-URI favicon.
 */
export function setFavicon(): void {
  const name: string = __CHECKOUT_NAME__;
  const suffix = name.includes("-") ? name.split("-").slice(1).join("-") : name;

  let svg: string | undefined;

  // Check emoji map first
  const emoji = emojiMap[suffix];
  if (emoji) {
    svg = emojiSvg(emoji);
  } else {
    svg = customSvgMap[suffix];
  }

  if (!svg) svg = emojiSvg("🤔");

  // Append a cache-busting fragment so browsers don't serve a stale favicon
  // after switching checkouts or restarting the dev server.
  const encoded =
    "data:image/svg+xml," + encodeURIComponent(svg) + "#v=" + Date.now();

  let link = document.querySelector<HTMLLinkElement>("link[rel='icon']");
  if (!link) {
    link = document.createElement("link");
    link.rel = "icon";
    document.head.appendChild(link);
  }
  link.type = "image/svg+xml";
  link.href = encoded;
}
