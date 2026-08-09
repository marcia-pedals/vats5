-- Initial schema: a single table used by the hello-world page.

CREATE TABLE hello_world (
  id SERIAL PRIMARY KEY,
  message TEXT NOT NULL,
  created_at TIMESTAMPTZ NOT NULL DEFAULT now()
);

INSERT INTO hello_world (message) VALUES
  ('Hello, world!'),
  ('Bonjour, monde!'),
  ('Hola, mundo!'),
  ('こんにちは世界'),
  ('Καλημέρα κόσμε');
