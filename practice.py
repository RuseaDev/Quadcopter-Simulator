import unittest 
class TestMyFunction(unittest.TestCase):
    def test_addition(self):
        result = 1+ 1
        self.assertEqual(result, 2)

if __name__ == '__main__':
    unittest.main()