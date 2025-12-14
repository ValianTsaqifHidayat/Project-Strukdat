# Project-Struktur Data by Valian Tsaqif Hidayat (5024241070)
# Final Project: Analisis Performa Algoritma Deteksi Tumbukan Partikel 2D

---

## 1. Overview Proyek

Proyek ini bertujuan untuk menganalisis dan membandingkan performa dua algoritma utama dalam mendeteksi tumbukan (kolisi) antar partikel dalam sebuah simulasi fisika 2D real-time: Brute Force dan Quadtree Spatial Partitioning. Simulasi dikembangkan menggunakan bahasa pemrograman C++ dengan framework SFML untuk visualisasi. Hasilnya memungkinkan pengguna untuk mengamati secara langsung bagaimana kompleksitas waktu algoritma ($O(N^2)$ vs. $O(N \log N)$) memengaruhi kelancaran (frame rate) aplikasi ketika jumlah partikel ($N$) ditingkatkan.

## 2. Fitur Utama

* **Simulasi Partikel Real-Time:** Partikel bergerak bebas (Verlet Integration) dengan posisi, kecepatan, radius, dan warna yang diinisialisasi secara acak.
* **Gerakan Abadi:** Mekanisme *minimum speed* diterapkan untuk memastikan partikel tidak berhenti total.
* **Visualisasi Statistik:** FPS, Waktu Pemrosesan Algoritma (ms), dan Jumlah Cek Tumbukan ditampilkan di judul jendela SFML.

## 3. Dua Metode Collision Detection

| Metode | Kompleksitas Waktu | Deskripsi |
| :--- | :--- | :--- |
| **Brute Force** | $O(N^2)$ | Membandingkan semua pasangan partikel ($\frac{N(N-1)}{2}$ cek per *frame*).  |
| **Quadtree Spatial Partitioning** | $O(N \log N)$ | Membagi ruang menjadi node rekursif untuk mempercepat pencarian, hanya memeriksa partikel di area terdekat.  |

## 4. Kontrol Interaktif (Real-Time Switching)

Kontrol ini digunakan saat simulasi sudah berjalan:

| Tombol | Fungsi | Hasil Kinerja |
| :--- | :--- | :--- |
| **Q** | **Ganti ke Quadtree** | Simulasi berjalan lancar (Performa $O(N \log N)$). |
| **B** | **Ganti ke Brute Force** | Simulasi akan *lag* pada $N$ tinggi (Performa $O(N^2)$). |

## 5. Persyaratan dan Kompilasi

Proyek ini menggunakan C++ dan membutuhkan SFML (Simple and Fast Multimedia Library).

### Persyaratan

* C++ Compiler (Disarankan GCC/G++ atau Clang)
* SFML Library (Versi 2.5 atau lebih baru)

### Kompilasi (Contoh menggunakan g++)

## Kesimpulan
Proyek simulasi fisika partikel 2D ini menyimpulkan bahwa pemilihan algoritma deteksi tumbukan sangat krusial untuk skalabilitas aplikasi real-time. Demonstrasi live switching antara algoritma Brute Force ($O(N^2)$) dan Quadtree Spatial Partitioning ($O(N \log N)$) secara dramatis memvalidasi perbedaan kompleksitas waktu. Saat jumlah partikel ($N$) tinggi, Brute Force menyebabkan bottleneck performa yang parah dan penurunan Frame Rate (FPS) akibat jumlah collision checks yang kuadratik. Sebaliknya, Quadtree secara efisien mengoptimalkan pencarian ruang, menjaga waktu pemrosesan minimal dan FPS tetap stabil, sehingga membuktikan superioritas teknik spatial partitioning untuk menangani masalah banyak objek (N-body problem) secara efisien dalam lingkungan simulasi dinamis.
