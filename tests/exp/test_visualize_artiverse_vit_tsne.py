from __future__ import annotations

import importlib.util
import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np
from PIL import Image


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "exp" / "scripts" / "visualize_artiverse_vit_tsne.py"
MODEL_SNAPSHOT = Path(
    "/root/.cache/huggingface/hub/models--facebook--dinov2-base/"
    "snapshots/f9e44c814b77203eaa57a6bdbbd535f21ede1415"
)
CLIP_MODEL_SNAPSHOT = Path(
    "/root/.cache/huggingface/hub/models--openai--clip-vit-base-patch32/"
    "snapshots/3d74acf9a28c67741b2f4f2ea7635f0aaf6f0268"
)


class ArtiverseVitTsneTests(unittest.TestCase):
    def load_subject(self):
        if not SCRIPT.is_file():
            self.fail(f"visualization script does not exist: {SCRIPT}")
        spec = importlib.util.spec_from_file_location("visualize_artiverse_vit_tsne", SCRIPT)
        assert spec is not None and spec.loader is not None
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)
        return module

    def require_callable(self, subject, name: str):
        value = getattr(subject, name, None)
        if not callable(value):
            self.fail(f"visualization script does not implement {name}()")
        return value

    def test_discovery_only_returns_sorted_reference_renders(self) -> None:
        subject = self.load_subject()
        with tempfile.TemporaryDirectory() as temporary:
            data = Path(temporary) / "data"
            first = data / "chair" / "source_b" / "model_2" / "imgs"
            second = data / "cabinet" / "source_a" / "model_1" / "imgs"
            texture = data / "chair" / "source_b" / "model_2" / "urdf_w_collider" / "textures"
            for directory in (first, second, texture):
                directory.mkdir(parents=True)
            for path in (first / "001.png", first / "000.png", second / "000.png", texture / "wood.png"):
                Image.new("RGB", (2, 2), "white").save(path)
            Image.new("RGB", (2, 2), "white").save(first / "preview.jpg")

            samples = subject.discover_render_samples(data)

            self.assertEqual(
                [sample.image_path.relative_to(data).as_posix() for sample in samples],
                [
                    "cabinet/source_a/model_1/imgs/000.png",
                    "chair/source_b/model_2/imgs/000.png",
                    "chair/source_b/model_2/imgs/001.png",
                ],
            )
            self.assertEqual(
                (samples[0].category, samples[0].source, samples[0].model_id, samples[0].view_id),
                ("cabinet", "source_a", "model_1", "000"),
            )

    def test_discovery_rejects_a_render_symlink_outside_the_data_root(self) -> None:
        subject = self.load_subject()
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            data = root / "data"
            image_dir = data / "chair" / "source" / "model" / "imgs"
            image_dir.mkdir(parents=True)
            outside = root / "outside.png"
            Image.new("RGB", (2, 2), "white").save(outside)
            (image_dir / "000.png").symlink_to(outside)

            with self.assertRaisesRegex(ValueError, "outside data directory"):
                subject.discover_render_samples(data)

    def test_picture_discovery_preserves_each_numbered_image(self) -> None:
        subject = self.load_subject()
        discover_picture_samples = self.require_callable(subject, "discover_picture_samples")
        with tempfile.TemporaryDirectory() as temporary:
            data = Path(temporary) / "picture"
            bench = data / "Bench" / "Wood Swing"
            grinder = data / "0611" / "manual_coffee_grinder"
            nested = bench / "nested"
            for directory in (bench, grinder, nested):
                directory.mkdir(parents=True)
            for path in (bench / "002.png", bench / "001.png", grinder / "003.png"):
                Image.new("RGB", (2, 2), "white").save(path)
            Image.new("RGB", (2, 2), "white").save(bench / "preview.jpg")
            Image.new("RGB", (2, 2), "white").save(nested / "004.png")

            samples = discover_picture_samples(data)

            self.assertEqual(
                [sample.image_path.relative_to(data).as_posix() for sample in samples],
                [
                    "0611/manual_coffee_grinder/003.png",
                    "Bench/Wood Swing/001.png",
                    "Bench/Wood Swing/002.png",
                ],
            )
            self.assertEqual(
                [
                    (sample.category, sample.subcategory, sample.image_id)
                    for sample in samples
                ],
                [
                    ("0611", "manual_coffee_grinder", "003"),
                    ("Bench", "Wood Swing", "001"),
                    ("Bench", "Wood Swing", "002"),
                ],
            )

    def test_picture_discovery_rejects_a_symlink_outside_the_data_root(self) -> None:
        subject = self.load_subject()
        discover_picture_samples = self.require_callable(subject, "discover_picture_samples")
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            data = root / "picture"
            image_dir = data / "Bench" / "Wood Swing"
            image_dir.mkdir(parents=True)
            outside = root / "outside.png"
            Image.new("RGB", (2, 2), "white").save(outside)
            (image_dir / "001.png").symlink_to(outside)

            with self.assertRaisesRegex(ValueError, "outside data directory"):
                discover_picture_samples(data)

    def test_picture_validation_counts_images_without_merging_subcategories(self) -> None:
        subject = self.load_subject()
        validate_picture_samples = self.require_callable(subject, "validate_picture_samples")
        picture_sample = self.require_callable(subject, "PictureSample")
        samples = [
            picture_sample(Path("Bench/Wood Swing/001.png"), "Bench", "Wood Swing", "001"),
            picture_sample(Path("Bench/Wood Swing/002.png"), "Bench", "Wood Swing", "002"),
            picture_sample(
                Path("0611/manual_coffee_grinder/003.png"),
                "0611",
                "manual_coffee_grinder",
                "003",
            ),
        ]

        summary = validate_picture_samples(samples)

        self.assertEqual(
            summary,
            {
                "image_count": 3,
                "category_count": 2,
                "semantic_category_count": 1,
                "subcategory_count": 2,
                "unmapped_batch_image_count": 1,
            },
        )

    def test_load_rgb_image_composites_transparency_onto_white(self) -> None:
        subject = self.load_subject()
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "render.png"
            image = Image.new("RGBA", (2, 1))
            image.putdata([(255, 0, 0, 255), (0, 0, 0, 0)])
            image.save(path)

            loaded = subject.load_rgb_image(path)

            self.assertEqual(loaded.mode, "RGB")
            self.assertEqual(loaded.getpixel((0, 0)), (255, 0, 0))
            self.assertEqual(loaded.getpixel((1, 0)), (255, 255, 255))

    def test_model_aggregation_normalizes_views_before_averaging(self) -> None:
        subject = self.load_subject()
        samples = [
            subject.RenderSample(Path("a/000.png"), "chair", "src", "a", "000"),
            subject.RenderSample(Path("a/001.png"), "chair", "src", "a", "001"),
            subject.RenderSample(Path("b/000.png"), "table", "src", "b", "000"),
        ]
        image_features = np.asarray([[2.0, 0.0], [0.0, 3.0], [0.0, 5.0]], dtype=np.float32)

        model_features, models = subject.aggregate_model_features(image_features, samples)

        np.testing.assert_allclose(
            model_features,
            np.asarray([[2**-0.5, 2**-0.5], [0.0, 1.0]], dtype=np.float32),
            rtol=1e-6,
            atol=1e-6,
        )
        self.assertEqual(
            [(model.category, model.source, model.model_id, model.view_count) for model in models],
            [("chair", "src", "a", 2), ("table", "src", "b", 1)],
        )

    def test_tsne_is_reproducible_and_uses_a_valid_small_sample_perplexity(self) -> None:
        subject = self.load_subject()
        features = np.random.default_rng(7).normal(size=(12, 6)).astype(np.float32)

        try:
            first, first_info = subject.compute_tsne(
                features,
                requested_perplexity=30.0,
                random_state=11,
                max_iter=250,
                n_jobs=1,
            )
            second, second_info = subject.compute_tsne(
                features,
                requested_perplexity=30.0,
                random_state=11,
                max_iter=250,
                n_jobs=1,
            )
        except TypeError as exc:
            self.fail(f"compute_tsne does not expose sklearn worker control: {exc}")

        self.assertEqual(first.shape, (12, 2))
        self.assertTrue(np.isfinite(first).all())
        np.testing.assert_allclose(first, second, rtol=0.0, atol=0.0)
        self.assertEqual(first_info, second_info)
        self.assertLess(first_info["perplexity"], 12)
        self.assertLessEqual(first_info["pca_components"], 6)
        self.assertEqual(first_info["n_jobs"], 1)

    def test_module_defaults_blas_to_one_thread_before_numpy_import(self) -> None:
        environment = {
            key: value
            for key, value in os.environ.items()
            if key not in {"OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"}
        }
        code = (
            "import importlib.util, json, sys; "
            f"spec=importlib.util.spec_from_file_location('subject', {str(SCRIPT)!r}); "
            "module=importlib.util.module_from_spec(spec); sys.modules[spec.name]=module; "
            "spec.loader.exec_module(module); "
            "from threadpoolctl import threadpool_info; "
            "print(json.dumps([item['num_threads'] for item in threadpool_info() "
            "if item['user_api'] == 'blas']))"
        )

        result = subprocess.run(
            [sys.executable, "-c", code],
            cwd=ROOT,
            env=environment,
            capture_output=True,
            text=True,
            check=False,
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        blas_threads = json.loads(result.stdout.strip().splitlines()[-1])
        self.assertTrue(blas_threads)
        self.assertEqual(blas_threads, [1] * len(blas_threads))

    def test_cli_caps_default_tsne_workers_at_64(self) -> None:
        subject = self.load_subject()

        args = subject.build_argument_parser().parse_args([])

        self.assertGreaterEqual(args.tsne_jobs, 1)
        self.assertLessEqual(args.tsne_jobs, 64)

    def test_legacy_dinov2_cache_keeps_its_encoder_label(self) -> None:
        subject = self.load_subject()
        encoder_label_from_extraction = self.require_callable(
            subject,
            "encoder_label_from_extraction",
        )

        label = encoder_label_from_extraction({"model_type": "dinov2"})

        self.assertEqual(label, "DINOv2")

    def test_plot_and_coordinate_table_are_written(self) -> None:
        subject = self.load_subject()
        coordinates = np.asarray([[0.0, 0.0], [1.0, 0.5], [-0.5, 1.0]], dtype=np.float32)
        records = [
            {"category": "chair", "source": "a", "model_id": "m1", "view_id": "000", "image_path": "p1"},
            {"category": "chair", "source": "b", "model_id": "m2", "view_id": "000", "image_path": "p2"},
            {"category": "table", "source": "a", "model_id": "m3", "view_id": "000", "image_path": "p3"},
        ]
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            plot_path = output / "plot.png"
            csv_path = output / "coordinates.csv"

            subject.save_embedding_plot(
                coordinates,
                [record["category"] for record in records],
                plot_path,
                title="Test embedding",
            )
            subject.write_coordinates_csv(coordinates, records, csv_path)

            self.assertTrue(plot_path.is_file())
            self.assertGreater(plot_path.stat().st_size, 1_000)
            with Image.open(plot_path) as image:
                self.assertGreater(image.width, 100)
                self.assertGreater(image.height, 100)
            lines = csv_path.read_text(encoding="utf-8").splitlines()
            self.assertEqual(lines[0], "tsne_x,tsne_y,category,source,model_id,view_id,image_path")
            self.assertEqual(len(lines), 4)

    def test_many_label_plot_uses_a_landscape_multi_column_legend(self) -> None:
        subject = self.load_subject()
        coordinates = np.column_stack(
            (np.linspace(-1.0, 1.0, 84), np.sin(np.linspace(0.0, 4.0, 84)))
        ).astype(np.float32)
        labels = [f"category_{index:02d}" for index in range(84)]
        with tempfile.TemporaryDirectory() as temporary:
            plot_path = Path(temporary) / "many_labels.png"

            subject.save_embedding_plot(
                coordinates,
                labels,
                plot_path,
                title="Many labels",
                legend_max_labels=100,
                dpi=60,
            )

            with Image.open(plot_path) as image:
                self.assertGreater(image.width, image.height)

    def test_full_leaf_plot_assigns_a_unique_color_to_each_label(self) -> None:
        subject = self.load_subject()
        coordinates = np.column_stack(
            (np.linspace(-1.0, 1.0, 451), np.sin(np.linspace(0.0, 8.0, 451)))
        ).astype(np.float32)
        labels = [f"category_{index:03d}/leaf" for index in range(451)]
        with tempfile.TemporaryDirectory() as temporary:
            colors = subject.save_embedding_plot(
                coordinates,
                labels,
                Path(temporary) / "full_leaf.png",
                title="Full leaf labels",
                legend_max_labels=0,
                distinct_colors=True,
                dpi=40,
            )

        self.assertEqual(len(colors), 451)
        self.assertEqual(len(set(colors.values())), 451)

    def test_picture_subcategory_labels_keep_0611_and_sparse_groups_unmapped(self) -> None:
        subject = self.load_subject()
        build_labels = self.require_callable(
            subject,
            "build_picture_subcategory_plot_labels",
        )
        picture_sample = self.require_callable(subject, "PictureSample")
        samples = []
        for image_index in range(5):
            samples.append(
                picture_sample(
                    Path(f"Bench/Wood Swing/{image_index:03d}.png"),
                    "Bench",
                    "Wood Swing",
                    f"{image_index:03d}",
                )
            )
            samples.append(
                picture_sample(
                    Path(f"0611/bookcase/{image_index:03d}.png"),
                    "0611",
                    "bookcase",
                    f"{image_index:03d}",
                )
            )
        for image_index in range(2):
            samples.append(
                picture_sample(
                    Path(f"Sign/sign/{image_index:03d}.png"),
                    "Sign",
                    "sign",
                    f"{image_index:03d}",
                )
            )

        labels = build_labels(samples, min_count=5)

        self.assertEqual(labels[:2], ["Bench/Wood Swing", "0611 (unmapped batch)"])
        self.assertEqual(labels[8:10], ["Bench/Wood Swing", "0611 (unmapped batch)"])
        self.assertEqual(labels[-2:], ["Other subcategories (n<5)"] * 2)

    def test_picture_leaf_labels_keep_every_scoped_subcategory_independent(self) -> None:
        subject = self.load_subject()
        build_labels = self.require_callable(
            subject,
            "build_picture_leaf_plot_labels",
        )
        picture_sample = self.require_callable(subject, "PictureSample")
        samples = [
            picture_sample(Path("0611/bookcase/001.png"), "0611", "bookcase", "001"),
            picture_sample(Path("Bench/Wood Swing/001.png"), "Bench", "Wood Swing", "001"),
            picture_sample(Path("Sign/sign/001.png"), "Sign", "sign", "001"),
        ]

        labels = build_labels(samples)

        self.assertEqual(
            labels,
            ["0611/bookcase", "Bench/Wood Swing", "Sign/sign"],
        )

    def test_picture_taxonomy_summary_counts_parent_and_leaf_nodes(self) -> None:
        subject = self.load_subject()
        build_summary = self.require_callable(
            subject,
            "build_picture_taxonomy_summary",
        )
        picture_sample = self.require_callable(subject, "PictureSample")
        samples = [
            picture_sample(Path("0611/bookcase/001.png"), "0611", "bookcase", "001"),
            picture_sample(Path("0611/grinder/001.png"), "0611", "grinder", "001"),
            picture_sample(Path("Bench/Wood Swing/001.png"), "Bench", "Wood Swing", "001"),
        ]

        summary = build_summary(samples)

        self.assertEqual(
            summary,
            {
                "category_count": 2,
                "semantic_category_count": 1,
                "leaf_subcategory_count": 3,
                "semantic_leaf_subcategory_count": 1,
                "taxonomy_node_count": 5,
                "semantic_taxonomy_node_count": 2,
            },
        )

    @unittest.skipUnless(MODEL_SNAPSHOT.is_dir(), "cached DINOv2 snapshot is unavailable")
    def test_extract_vit_features_returns_normalized_dinov2_cls_embedding(self) -> None:
        subject = self.load_subject()
        extract_vit_features = self.require_callable(subject, "extract_vit_features")
        with tempfile.TemporaryDirectory() as temporary:
            image_path = Path(temporary) / "render.png"
            Image.new("RGBA", (64, 64), (40, 120, 200, 255)).save(image_path)
            samples = [subject.RenderSample(image_path, "chair", "source", "model", "000")]

            features, info = extract_vit_features(
                samples,
                model_name_or_path=MODEL_SNAPSHOT,
                batch_size=1,
                device="cpu",
                num_workers=1,
                use_amp=False,
            )

            self.assertEqual(features.shape, (1, 768))
            self.assertAlmostEqual(float(np.linalg.norm(features[0])), 1.0, places=5)
            self.assertEqual(info["sample_count"], 1)
            self.assertEqual(info["feature_dim"], 768)
            self.assertEqual(info["device"], "cpu")

    @unittest.skipUnless(CLIP_MODEL_SNAPSHOT.is_dir(), "cached CLIP snapshot is unavailable")
    def test_extract_vit_features_returns_normalized_clip_projection_embedding(self) -> None:
        subject = self.load_subject()
        extract_vit_features = self.require_callable(subject, "extract_vit_features")
        with tempfile.TemporaryDirectory() as temporary:
            image_path = Path(temporary) / "render.png"
            Image.new("RGB", (64, 64), (180, 90, 30)).save(image_path)
            samples = [subject.RenderSample(image_path, "chair", "source", "model", "000")]

            features, info = extract_vit_features(
                samples,
                model_name_or_path=CLIP_MODEL_SNAPSHOT,
                batch_size=1,
                device="cpu",
                num_workers=1,
                use_amp=False,
            )

            self.assertEqual(features.shape, (1, 512))
            self.assertAlmostEqual(float(np.linalg.norm(features[0])), 1.0, places=5)
            self.assertEqual(info["model_type"], "clip")
            self.assertEqual(info["feature_source"], "visual_projection")
            self.assertEqual(info["encoder_label"], "CLIP ViT-B/32")

    def test_extract_vit_features_requires_an_existing_local_model(self) -> None:
        subject = self.load_subject()
        extract_vit_features = self.require_callable(subject, "extract_vit_features")
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            image_path = root / "render.png"
            Image.new("RGB", (2, 2), "white").save(image_path)
            sample = subject.RenderSample(image_path, "chair", "source", "model", "000")

            try:
                extract_vit_features(
                    [sample],
                    model_name_or_path=root / "missing-model",
                    batch_size=1,
                    device="cpu",
                    num_workers=1,
                    use_amp=False,
                )
            except Exception as exc:
                self.assertIsInstance(exc, FileNotFoundError)
                self.assertIn("local model", str(exc))
            else:
                self.fail("missing local model directory was accepted")

    def test_create_visualizations_writes_image_and_model_artifacts(self) -> None:
        subject = self.load_subject()
        create_visualizations = self.require_callable(subject, "create_visualizations")
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            data = root / "data"
            output = root / "output"
            samples = []
            for model_index in range(4):
                category = "chair" if model_index < 2 else "table"
                source = "source_a" if model_index % 2 == 0 else "source_b"
                for view_index in range(2):
                    samples.append(
                        subject.RenderSample(
                            data / category / source / f"model_{model_index}" / "imgs" / f"{view_index:03d}.png",
                            category,
                            source,
                            f"model_{model_index}",
                            f"{view_index:03d}",
                        )
                    )
            features = np.random.default_rng(3).normal(size=(8, 6)).astype(np.float32)

            summary = create_visualizations(
                features,
                samples,
                data_dir=data,
                output_dir=output,
                encoder_label="CLIP ViT-B/32",
                requested_perplexity=30.0,
                random_state=17,
                max_iter=250,
                plot_dpi=60,
            )

            expected_files = {
                "image_tsne_by_category.png",
                "image_tsne_by_source.png",
                "image_tsne_coordinates.csv",
                "model_features.npy",
                "model_tsne_by_category.png",
                "model_tsne_by_source.png",
                "model_tsne_coordinates.csv",
                "visualization_summary.json",
            }
            self.assertTrue(expected_files.issubset({path.name for path in output.iterdir()}))
            self.assertEqual(summary["image_count"], 8)
            self.assertEqual(summary["model_count"], 4)
            self.assertEqual(summary["encoder_label"], "CLIP ViT-B/32")
            persisted = json.loads((output / "visualization_summary.json").read_text(encoding="utf-8"))
            self.assertEqual(persisted, summary)

    def test_create_picture_visualizations_writes_unaggregated_image_artifacts(self) -> None:
        subject = self.load_subject()
        create_picture_visualizations = self.require_callable(
            subject,
            "create_picture_visualizations",
        )
        picture_sample = self.require_callable(subject, "PictureSample")
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            data = root / "picture"
            output = root / "output"
            samples = [
                picture_sample(
                    data / "Bench" / "Wood Swing" / "001.png",
                    "Bench",
                    "Wood Swing",
                    "001",
                ),
                picture_sample(
                    data / "Bench" / "Wood Swing" / "002.png",
                    "Bench",
                    "Wood Swing",
                    "002",
                ),
                picture_sample(data / "Sign" / "sign" / "001.png", "Sign", "sign", "001"),
                picture_sample(data / "Sign" / "sign" / "002.png", "Sign", "sign", "002"),
                picture_sample(
                    data / "0611" / "manual_coffee_grinder" / "001.png",
                    "0611",
                    "manual_coffee_grinder",
                    "001",
                ),
                picture_sample(
                    data / "0611" / "manual_coffee_grinder" / "002.png",
                    "0611",
                    "manual_coffee_grinder",
                    "002",
                ),
            ]
            features = np.random.default_rng(5).normal(size=(6, 4)).astype(np.float32)

            summary = create_picture_visualizations(
                features,
                samples,
                data_dir=data,
                output_dir=output,
                encoder_label="CLIP ViT-B/32",
                requested_perplexity=30.0,
                random_state=23,
                max_iter=250,
                plot_dpi=40,
            )

            expected_files = {
                "image_tsne_by_category.png",
                "image_tsne_by_subcategory.png",
                "image_tsne_by_subcategory_grouped.png",
                "image_tsne_coordinates.csv",
                "image_tsne_class_index.csv",
                "visualization_summary.json",
            }
            self.assertEqual({path.name for path in output.iterdir()}, expected_files)
            self.assertEqual(summary["image_count"], 6)
            self.assertEqual(summary["category_count"], 3)
            self.assertEqual(summary["subcategory_count"], 3)
            self.assertEqual(summary["leaf_subcategory_count"], 3)
            self.assertEqual(summary["taxonomy_node_count"], 6)
            self.assertEqual(summary["displayed_subcategory_label_count"], 3)
            self.assertNotIn("model_count", summary)
            csv_lines = (output / "image_tsne_coordinates.csv").read_text(
                encoding="utf-8"
            ).splitlines()
            self.assertEqual(
                csv_lines[0],
                "tsne_x,tsne_y,category,subcategory,image_id,image_path",
            )
            self.assertEqual(len(csv_lines), 7)
            self.assertIn("Bench/Wood Swing/001.png", csv_lines[1])
            self.assertIn("Bench/Wood Swing/002.png", csv_lines[2])
            class_index_lines = (output / "image_tsne_class_index.csv").read_text(
                encoding="utf-8"
            ).splitlines()
            self.assertEqual(
                class_index_lines[0],
                "label,category,subcategory,image_count,color",
            )
            self.assertEqual(len(class_index_lines), 4)
            self.assertIn("0611/manual_coffee_grinder", class_index_lines[1])
            self.assertFalse((output / "model_features.npy").exists())

    def test_cli_supports_explicit_picture_dataset_format(self) -> None:
        subject = self.load_subject()

        args = subject.build_argument_parser().parse_args(["--dataset-format", "picture"])

        self.assertEqual(args.dataset_format, "picture")

    def test_dataset_defaults_isolate_picture_from_artiverse(self) -> None:
        subject = self.load_subject()
        resolve_dataset_paths = self.require_callable(subject, "resolve_dataset_paths")

        artiverse_paths = resolve_dataset_paths("artiverse", None, None)
        picture_paths = resolve_dataset_paths("picture", None, None)
        explicit = resolve_dataset_paths(
            "picture",
            Path("custom-data"),
            Path("custom-output"),
        )

        self.assertEqual(
            artiverse_paths,
            (subject.DEFAULT_DATA_DIR, subject.DEFAULT_OUTPUT_DIR),
        )
        self.assertEqual(
            picture_paths,
            (subject.DEFAULT_PICTURE_DATA_DIR, subject.DEFAULT_PICTURE_OUTPUT_DIR),
        )
        self.assertEqual(explicit, (Path("custom-data"), Path("custom-output")))

    def test_dataset_validation_requires_the_expected_views_per_model(self) -> None:
        subject = self.load_subject()
        validate_samples = self.require_callable(subject, "validate_artiverse_samples")
        samples = [
            subject.RenderSample(Path("a/000.png"), "chair", "src", "a", "000"),
            subject.RenderSample(Path("a/001.png"), "chair", "src", "a", "001"),
            subject.RenderSample(Path("b/000.png"), "table", "src", "b", "000"),
            subject.RenderSample(Path("b/001.png"), "table", "src", "b", "001"),
        ]

        summary = validate_samples(samples, expected_views=2)

        self.assertEqual(
            summary,
            {"image_count": 4, "model_count": 2, "category_count": 2, "source_count": 1, "views_per_model": 2},
        )
        with self.assertRaisesRegex(ValueError, "expected 2 views"):
            validate_samples(samples[:-1], expected_views=2)

    def test_feature_cache_rejects_a_changed_sample_order(self) -> None:
        subject = self.load_subject()
        save_feature_cache = self.require_callable(subject, "save_feature_cache")
        load_feature_cache = self.require_callable(subject, "load_feature_cache")
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            data = root / "data"
            output = root / "output"
            model = root / "model"
            model.mkdir()
            (model / "config.json").write_text('{"model_type":"test"}\n', encoding="utf-8")
            samples = [
                subject.RenderSample(data / "chair/src/a/imgs/000.png", "chair", "src", "a", "000"),
                subject.RenderSample(data / "table/src/b/imgs/000.png", "table", "src", "b", "000"),
            ]
            for sample in samples:
                sample.image_path.parent.mkdir(parents=True, exist_ok=True)
                Image.new("RGB", (2, 2), "white").save(sample.image_path)
            features = np.asarray([[1.0, 0.0], [0.0, 1.0]], dtype=np.float32)
            extraction = {"feature_dim": 2, "sample_count": 2, "device": "cpu"}

            manifest = save_feature_cache(
                features,
                samples,
                output_dir=output,
                data_dir=data,
                model_name_or_path=model,
                extraction_info=extraction,
            )
            loaded, loaded_manifest = load_feature_cache(
                output_dir=output,
                samples=samples,
                data_dir=data,
                model_name_or_path=model,
            )

            np.testing.assert_array_equal(loaded, features)
            self.assertEqual(loaded_manifest, manifest)
            with self.assertRaisesRegex(RuntimeError, "sample fingerprint"):
                load_feature_cache(
                    output_dir=output,
                    samples=list(reversed(samples)),
                    data_dir=data,
                    model_name_or_path=model,
                )

            (model / "config.json").write_text('{"model_type":"changed"}\n', encoding="utf-8")
            with self.assertRaisesRegex(RuntimeError, "model fingerprint"):
                load_feature_cache(
                    output_dir=output,
                    samples=samples,
                    data_dir=data,
                    model_name_or_path=model,
                )
            (model / "config.json").write_text('{"model_type":"test"}\n', encoding="utf-8")

            Image.new("RGB", (5, 5), "black").save(samples[0].image_path)
            with self.assertRaisesRegex(RuntimeError, "sample fingerprint"):
                load_feature_cache(
                    output_dir=output,
                    samples=samples,
                    data_dir=data,
                    model_name_or_path=model,
                )

    def test_picture_cache_rejects_same_stat_changed_image_content(self) -> None:
        subject = self.load_subject()
        save_feature_cache = self.require_callable(subject, "save_feature_cache")
        load_feature_cache = self.require_callable(subject, "load_feature_cache")
        picture_sample = self.require_callable(subject, "PictureSample")
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            data = root / "picture"
            output = root / "output"
            model = root / "model"
            image_path = data / "Bench" / "Wood Swing" / "001.png"
            image_path.parent.mkdir(parents=True)
            image_path.write_bytes(b"A" * 64)
            model.mkdir()
            (model / "config.json").write_text('{"model_type":"test"}\n', encoding="utf-8")
            samples = [picture_sample(image_path, "Bench", "Wood Swing", "001")]
            features = np.asarray([[1.0, 0.0]], dtype=np.float32)
            extraction = {"feature_dim": 2, "sample_count": 1, "device": "cpu"}

            save_feature_cache(
                features,
                samples,
                output_dir=output,
                data_dir=data,
                model_name_or_path=model,
                extraction_info=extraction,
            )
            original_stat = image_path.stat()
            image_path.write_bytes(b"B" * 64)
            os.utime(
                image_path,
                ns=(original_stat.st_atime_ns, original_stat.st_mtime_ns),
            )
            changed_stat = image_path.stat()
            self.assertEqual(changed_stat.st_size, original_stat.st_size)
            self.assertEqual(changed_stat.st_mtime_ns, original_stat.st_mtime_ns)

            with self.assertRaisesRegex(RuntimeError, "sample fingerprint"):
                load_feature_cache(
                    output_dir=output,
                    samples=samples,
                    data_dir=data,
                    model_name_or_path=model,
                )

    @unittest.skipUnless(MODEL_SNAPSHOT.is_dir(), "cached DINOv2 snapshot is unavailable")
    def test_run_pipeline_extracts_then_reuses_the_feature_cache(self) -> None:
        subject = self.load_subject()
        run_pipeline = self.require_callable(subject, "run_pipeline")
        pipeline_config = self.require_callable(subject, "PipelineConfig")
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            data = root / "data"
            output = root / "output"
            for model_index in range(4):
                category = "chair" if model_index < 2 else "table"
                source = "source_a" if model_index % 2 == 0 else "source_b"
                image_dir = data / category / source / f"model_{model_index}" / "imgs"
                image_dir.mkdir(parents=True)
                for view_index in range(2):
                    color = (40 * model_index, 60 * view_index, 180, 255)
                    Image.new("RGBA", (64, 64), color).save(image_dir / f"{view_index:03d}.png")
            common = {
                "data_dir": data,
                "output_dir": output,
                "model_name_or_path": MODEL_SNAPSHOT,
                "batch_size": 8,
                "device": "cpu",
                "num_workers": 1,
                "use_amp": False,
                "expected_views": 2,
                "requested_perplexity": 30.0,
                "random_state": 19,
                "tsne_max_iter": 250,
                "plot_dpi": 40,
                "tsne_verbose": 0,
            }

            first = run_pipeline(pipeline_config(**common, force_extract=True))
            feature_path = output / "image_features.npy"
            first_mtime = feature_path.stat().st_mtime_ns
            second = run_pipeline(pipeline_config(**common, force_extract=False))

            self.assertFalse(first["feature_cache_reused"])
            self.assertTrue(second["feature_cache_reused"])
            self.assertEqual(first_mtime, feature_path.stat().st_mtime_ns)
            self.assertEqual(second["dataset"]["image_count"], 8)
            self.assertEqual(second["dataset"]["model_count"], 4)
            self.assertTrue((output / "run_manifest.json").is_file())

            manifest_path = output / "feature_manifest.json"
            stale_manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            stale_manifest["sample_fingerprint"] = "stale"
            manifest_path.write_text(json.dumps(stale_manifest), encoding="utf-8")
            try:
                third = run_pipeline(pipeline_config(**common, force_extract=False))
            except RuntimeError as exc:
                self.fail(f"stale cache aborted the pipeline instead of re-extracting: {exc}")
            self.assertFalse(third["feature_cache_reused"])
            repaired = json.loads(manifest_path.read_text(encoding="utf-8"))
            self.assertNotEqual(repaired["sample_fingerprint"], "stale")

    @unittest.skipUnless(MODEL_SNAPSHOT.is_dir(), "cached DINOv2 snapshot is unavailable")
    def test_run_pipeline_picture_mode_keeps_numbered_images_independent(self) -> None:
        subject = self.load_subject()
        run_pipeline = self.require_callable(subject, "run_pipeline")
        pipeline_config = self.require_callable(subject, "PipelineConfig")
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            data = root / "picture"
            output = root / "output"
            paths = [
                data / "Bench" / "Wood Swing" / "001.png",
                data / "Bench" / "Wood Swing" / "002.png",
                data / "Sign" / "sign" / "001.png",
                data / "0611" / "manual_coffee_grinder" / "001.png",
            ]
            for image_index, path in enumerate(paths):
                path.parent.mkdir(parents=True, exist_ok=True)
                Image.new(
                    "RGB",
                    (64, 64),
                    (40 * image_index, 90, 180),
                ).save(path)
            common = {
                "dataset_format": "picture",
                "data_dir": data,
                "output_dir": output,
                "model_name_or_path": MODEL_SNAPSHOT,
                "batch_size": 4,
                "device": "cpu",
                "num_workers": 1,
                "use_amp": False,
                "expected_views": 999,
                "requested_perplexity": 30.0,
                "random_state": 29,
                "tsne_max_iter": 250,
                "plot_dpi": 40,
                "tsne_verbose": 0,
            }

            first = run_pipeline(pipeline_config(**common, force_extract=True))
            feature_path = output / "image_features.npy"
            first_mtime = feature_path.stat().st_mtime_ns
            second = run_pipeline(pipeline_config(**common, force_extract=False))

            self.assertFalse(first["feature_cache_reused"])
            self.assertTrue(second["feature_cache_reused"])
            self.assertEqual(first_mtime, feature_path.stat().st_mtime_ns)
            self.assertEqual(second["config"]["dataset_format"], "picture")
            self.assertEqual(second["dataset"]["image_count"], 4)
            self.assertEqual(second["dataset"]["subcategory_count"], 3)
            self.assertNotIn("model_count", second["visualization"])
            csv_lines = (output / "image_tsne_coordinates.csv").read_text(
                encoding="utf-8"
            ).splitlines()
            self.assertEqual(len(csv_lines), 5)
            self.assertTrue(any("Bench/Wood Swing/001.png" in line for line in csv_lines))
            self.assertTrue(any("Bench/Wood Swing/002.png" in line for line in csv_lines))
            self.assertFalse((output / "model_features.npy").exists())

    def test_run_pipeline_rejects_an_output_from_another_dataset_format(self) -> None:
        subject = self.load_subject()
        run_pipeline = self.require_callable(subject, "run_pipeline")
        pipeline_config = self.require_callable(subject, "PipelineConfig")
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            data = root / "picture"
            output = root / "output"
            for image_index in range(3):
                path = data / "Bench" / "Wood Swing" / f"{image_index:03d}.png"
                path.parent.mkdir(parents=True, exist_ok=True)
                Image.new("RGB", (2, 2), "white").save(path)
            output.mkdir()
            (output / "run_manifest.json").write_text(
                json.dumps({"config": {"dataset_format": "artiverse"}}),
                encoding="utf-8",
            )
            config = pipeline_config(
                dataset_format="picture",
                data_dir=data,
                output_dir=output,
                model_name_or_path=root / "missing-model",
                batch_size=1,
                device="cpu",
                num_workers=1,
                use_amp=False,
                expected_views=16,
                requested_perplexity=30.0,
                random_state=31,
                tsne_max_iter=250,
                plot_dpi=40,
                tsne_verbose=0,
                force_extract=True,
            )

            with self.assertRaisesRegex(ValueError, "belongs to artiverse"):
                run_pipeline(config)


if __name__ == "__main__":
    unittest.main()
